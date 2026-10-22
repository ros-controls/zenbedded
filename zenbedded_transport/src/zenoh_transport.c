// Copyright 2026 Open Source Robotics Foundation, Inc.
// Licensed under the Apache License, Version 2.0

#include "zenbedded_transport/zenoh_transport.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <zenoh-pico.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include "zenbedded_transport/rihs_ledger.h"

#if defined(CONFIG_ZENBEDDED_TRANSPORT_LOG_LEVEL)
LOG_MODULE_REGISTER(zenoh_transport, CONFIG_ZENBEDDED_TRANSPORT_LOG_LEVEL);
#else
LOG_MODULE_REGISTER(zenoh_transport, LOG_LEVEL_INF);
#endif

static z_owned_session_t z_session;
static uint32_t current_domain_id = 0;
static char current_node_name[CONFIG_ZENBEDDED_MAX_NODE_NAME_LEN] = {0};
static atomic_t transport_ref_count = ATOMIC_INIT(0);

#define MAX_PUBLISHERS CONFIG_ZENBEDDED_MAX_PUBLISHERS
#define MAX_SUBSCRIBERS CONFIG_ZENBEDDED_MAX_SUBSCRIBERS
#define RMW_GID_SIZE 16

#if defined(CONFIG_ZENBEDDED_TIER_1)
/* ROS 2 / rmw_zenoh Key Expression and Liveliness Overhead Constants */
#define LV_NODE_FIXED_OVERHEAD 54 /* "@ros2_lv//<32-hex-zid>/0/0/NN/%/%/" */
#define LV_PUB_SUB_FIXED_OVERHEAD \
  71 /* "@ros2_lv//<32-hex-zid>/0/10/{MP,MS}/%/%//%///::,7:,:,:,," */
#define TOPIC_KEYEXPR_FIXED_OVERHEAD \
  3                             /* Three '/' separators: "<domain_id>/<topic>/<type>/<hash>" */
#define MAX_DOMAIN_ID_DIGITS 10 /* Full uint32_t range (ROS 2 standard 0-232 uses 3 digits) */
#define RIHS01_TYPE_HASH_LEN 71 /* "RIHS01_" (7) + 64 hex chars */

#define NODE_LV_KEYEXPR_MAX_LEN \
  (LV_NODE_FIXED_OVERHEAD + MAX_DOMAIN_ID_DIGITS + CONFIG_ZENBEDDED_MAX_NODE_NAME_LEN + 1)

#define TOPIC_KEYEXPR_MAX_LEN                                                                  \
  (TOPIC_KEYEXPR_FIXED_OVERHEAD + MAX_DOMAIN_ID_DIGITS + CONFIG_ZENBEDDED_MAX_TOPIC_NAME_LEN + \
   CONFIG_ZENBEDDED_MAX_TYPE_NAME_LEN + RIHS01_TYPE_HASH_LEN + 1)

#define LV_KEYEXPR_MAX_LEN                                                                 \
  (LV_PUB_SUB_FIXED_OVERHEAD + MAX_DOMAIN_ID_DIGITS + CONFIG_ZENBEDDED_MAX_NODE_NAME_LEN + \
   CONFIG_ZENBEDDED_MAX_TOPIC_NAME_LEN + CONFIG_ZENBEDDED_MAX_TYPE_NAME_LEN +              \
   RIHS01_TYPE_HASH_LEN + 1)

#elif defined(CONFIG_ZENBEDDED_TIER_2)

#define TOPIC_KEYEXPR_MAX_LEN CONFIG_ZENBEDDED_MAX_TOPIC_NAME_LEN

#endif

#if defined(CONFIG_ZENBEDDED_TIER_1)
typedef struct __attribute__((packed))
{
  int64_t sequence_number;
  int64_t timestamp_ns;
  uint8_t gid_length;
  uint8_t gid[RMW_GID_SIZE];
} rmw_zenoh_attachment_t;
#endif

struct zenbedded_pub_s
{
  bool in_use;
  z_owned_publisher_t z_pub;
  char topic_keyexpr[TOPIC_KEYEXPR_MAX_LEN];
#if defined(CONFIG_ZENBEDDED_TIER_1)
  rmw_zenoh_attachment_t attachment;
  z_owned_liveliness_token_t lv_token;
  char lv_keyexpr[LV_KEYEXPR_MAX_LEN];
#endif
};

struct zenbedded_sub_s
{
  bool in_use;
  z_owned_subscriber_t z_sub;
  zenbedded_recv_cb_t user_cb;
  void * user_data;
  char topic_keyexpr[TOPIC_KEYEXPR_MAX_LEN];
#if defined(CONFIG_ZENBEDDED_TIER_1)
  z_owned_liveliness_token_t lv_token;
  char lv_keyexpr[LV_KEYEXPR_MAX_LEN];
#endif
};

static struct zenbedded_pub_s pub_pool[MAX_PUBLISHERS] = {0};
static struct zenbedded_sub_s sub_pool[MAX_SUBSCRIBERS] = {0};

#if defined(CONFIG_ZENBEDDED_TIER_1)
static z_owned_liveliness_token_t node_lv_token;
static char node_lv_str[NODE_LV_KEYEXPR_MAX_LEN] = {0};
#endif

static void internal_sub_handler(z_loaned_sample_t * sample, void * arg)
{
  struct zenbedded_sub_s * sub = (struct zenbedded_sub_s *)arg;
  if (!sub || !sub->user_cb)
  {
    return;
  }

  const z_loaned_bytes_t * payload = z_sample_payload(sample);
  size_t len = z_bytes_len(payload);

#if defined(CONFIG_ZENBEDDED_TIER_1)
  static uint8_t rx_buf[CONFIG_ZENBEDDED_TIER1_MAX_RX_BUFFER_SIZE];
  if (len == 0 || len > sizeof(rx_buf))
  {
    return;
  }

  z_bytes_reader_t reader = z_bytes_get_reader(payload);
  z_bytes_reader_read(&reader, rx_buf, len);
  sub->user_cb(rx_buf, len, sub->user_data);

#elif defined(CONFIG_ZENBEDDED_TIER_2)
  if (len == 0 || len > CONFIG_ZENBEDDED_TIER2_MAX_RX_BUFFER_SIZE)
  {
    return;
  }

  static uint8_t t2_rx_buf[CONFIG_ZENBEDDED_TIER2_MAX_RX_BUFFER_SIZE];

  z_bytes_reader_t reader = z_bytes_get_reader(payload);
  z_bytes_reader_read(&reader, t2_rx_buf, len);

  sub->user_cb(t2_rx_buf, len, sub->user_data);
#endif
}

zenbedded_mode_t zenbedded_mode_from_str(const char * mode_str)
{
  if (mode_str && strcmp(mode_str, "peer") == 0)
  {
    return ZENBEDDED_MODE_PEER;
  }
  return ZENBEDDED_MODE_CLIENT;
}

const char * zenbedded_mode_to_str(zenbedded_mode_t mode)
{
  return (mode == ZENBEDDED_MODE_PEER) ? "peer" : "client";
}

int zenbedded_transport_init(
  uint32_t domain_id, const char * node_name, zenbedded_mode_t mode, const char * locator)
{
  if (atomic_inc(&transport_ref_count) > 0)
  {
    LOG_DBG("Transport init ref count incremented: %d", (int)atomic_get(&transport_ref_count));
    return 0;
  }

  current_domain_id = domain_id;
  strncpy(current_node_name, node_name, sizeof(current_node_name) - 1);
  current_node_name[sizeof(current_node_name) - 1] = '\0';

  z_owned_config_t z_config;
  z_config_default(&z_config);
  zp_config_insert(z_config_loan_mut(&z_config), Z_CONFIG_MODE_KEY, zenbedded_mode_to_str(mode));

  static char full_locator[CONFIG_ZENBEDDED_MAX_LOCATOR_LEN];

  if (locator && strlen(locator) > 0)
  {
#if defined(CONFIG_ZENBEDDED_QOS_RELIABLE)
    snprintf(full_locator, sizeof(full_locator), "tcp/%s", locator);
#else
    snprintf(full_locator, sizeof(full_locator), "udp/%s", locator);
#endif

#if defined(CONFIG_ZENBEDDED_MODE_CLIENT)
    uint8_t key = Z_CONFIG_CONNECT_KEY;
#else
    uint8_t key = Z_CONFIG_LISTEN_KEY;
#endif

    zp_config_insert(z_config_loan_mut(&z_config), key, full_locator);

    LOG_INF("Transport Locator mapped to: %s (Key: %d)", full_locator, key);
  }

  if (z_open(&z_session, z_move(z_config), NULL) != Z_OK)
  {
    LOG_ERR("Failed to open Zenoh session!");
    atomic_set(&transport_ref_count, 0);
    return -1;
  }
  LOG_INF("Zenoh session opened successfully");

  zp_task_read_options_t read_opts;
  zp_task_read_options_default(&read_opts);
  zp_start_read_task(z_session_loan_mut(&z_session), &read_opts);

  zp_task_lease_options_t lease_opts;
  zp_task_lease_options_default(&lease_opts);
  zp_start_lease_task(z_session_loan_mut(&z_session), &lease_opts);

#ifdef CONFIG_ZENBEDDED_TIER_1
  z_id_t zid = z_info_zid(z_session_loan(&z_session));

  snprintf(
    node_lv_str, sizeof(node_lv_str),
    "@ros2_lv/%u/%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x/0/0/NN/%%/%%/%s",
    current_domain_id, zid.id[0], zid.id[1], zid.id[2], zid.id[3], zid.id[4], zid.id[5], zid.id[6],
    zid.id[7], zid.id[8], zid.id[9], zid.id[10], zid.id[11], zid.id[12], zid.id[13], zid.id[14],
    zid.id[15], current_node_name);

  LOG_DBG("[WIRE-TRACE] Transmitting Node Liveliness Declaration:");
  LOG_DBG("[WIRE-TRACE] %s", node_lv_str);

  z_view_keyexpr_t ke;
  z_view_keyexpr_from_str(&ke, node_lv_str);
  z_liveliness_declare_token(
    z_session_loan(&z_session), &node_lv_token, z_view_keyexpr_loan(&ke), NULL);
#endif

  LOG_INF("Transport Initialized on Domain %u (Node: %s)", domain_id, current_node_name);
  return 0;
}

int zenbedded_transport_undeclare_publisher(zenbedded_pub_t pub)
{
  if (!pub || !pub->in_use)
  {
    return -EINVAL;
  }

#if defined(CONFIG_ZENBEDDED_TIER_1)
  z_liveliness_undeclare_token(z_move(pub->lv_token));
#endif
  z_undeclare_publisher(z_publisher_move(&pub->z_pub));
  pub->in_use = false;
  LOG_INF("Undeclared Publisher: %s", pub->topic_keyexpr);
  return 0;
}

int zenbedded_transport_undeclare_subscriber(zenbedded_sub_t sub)
{
  if (!sub || !sub->in_use)
  {
    return -EINVAL;
  }

#if defined(CONFIG_ZENBEDDED_TIER_1)
  z_liveliness_undeclare_token(z_move(sub->lv_token));
#endif
  z_undeclare_subscriber(z_subscriber_move(&sub->z_sub));
  sub->in_use = false;
  LOG_INF("Undeclared Subscriber: %s", sub->topic_keyexpr);
  return 0;
}

void zenbedded_transport_destroy(void)
{
  atomic_val_t prev = atomic_dec(&transport_ref_count);
  if (prev <= 1)
  {
    atomic_set(&transport_ref_count, 0);
  }
  else
  {
    LOG_DBG(
      "Transport destroy: active references remaining (%d)", (int)atomic_get(&transport_ref_count));
    return;
  }

  for (int i = 0; i < MAX_PUBLISHERS; i++)
  {
    if (pub_pool[i].in_use)
    {
      zenbedded_transport_undeclare_publisher(&pub_pool[i]);
    }
  }
  for (int i = 0; i < MAX_SUBSCRIBERS; i++)
  {
    if (sub_pool[i].in_use)
    {
      zenbedded_transport_undeclare_subscriber(&sub_pool[i]);
    }
  }

#if defined(CONFIG_ZENBEDDED_TIER_1)
  z_liveliness_undeclare_token(z_move(node_lv_token));
#endif

  zp_stop_read_task(z_session_loan_mut(&z_session));
  zp_stop_lease_task(z_session_loan_mut(&z_session));

  z_close(z_session_loan_mut(&z_session), NULL);
  LOG_INF("Transport Destroyed Successfully");
}

void zenbedded_transport_spin(void)
{
  // Deprecated: Network I/O handled autonomously by Zenoh-Pico background tasks
}

#if defined(CONFIG_ZENBEDDED_TIER_1)
static void mangle_topic(char * dst, size_t dst_len, const char * src)
{
  if (src[0] == '/')
  {
    src++;
  }
  size_t i = 0;
  for (; i < dst_len - 1 && src[i] != '\0'; i++)
  {
    dst[i] = (src[i] == '/') ? '%' : src[i];
  }
  dst[i] = '\0';
}
#endif

zenbedded_pub_t zenbedded_transport_declare_publisher(
  const char * topic_name, const char * type_name)
{
  if (atomic_get(&transport_ref_count) <= 0)
  {
    return NULL;
  }

  struct zenbedded_pub_s * pub = NULL;
  for (int i = 0; i < MAX_PUBLISHERS; i++)
  {
    if (!pub_pool[i].in_use)
    {
      pub = &pub_pool[i];
      pub->in_use = true;
      break;
    }
  }
  if (!pub)
  {
    LOG_ERR("Publisher pool exhausted! Max limit: %d", MAX_PUBLISHERS);
    return NULL;
  }

  const char * clean_topic = (topic_name[0] == '/') ? topic_name + 1 : topic_name;
  z_view_keyexpr_t ke;

#if defined(CONFIG_ZENBEDDED_TIER_1)
  if (type_name == NULL)
  {
    LOG_ERR("Publisher declaration failed: type_name cannot be NULL in Tier 1");
    pub->in_use = false;
    return NULL;
  }
  const char * type_hash = zenbedded_get_rihs_hash(type_name);
  if (type_hash == NULL)
  {
    LOG_ERR(
      "Publisher declaration failed: unknown message type '%s' (not in RIHS ledger)", type_name);
    pub->in_use = false;
    return NULL;
  }
  snprintf(
    pub->topic_keyexpr, sizeof(pub->topic_keyexpr), "%u/%s/%s/%s", current_domain_id, clean_topic,
    type_name, type_hash);
  z_view_keyexpr_from_str_unchecked(&ke, pub->topic_keyexpr);

  if (
    z_declare_publisher(z_session_loan(&z_session), &pub->z_pub, z_view_keyexpr_loan(&ke), NULL) !=
    Z_OK)
  {
    pub->in_use = false;
    return NULL;
  }
#elif defined(CONFIG_ZENBEDDED_TIER_2)
  snprintf(pub->topic_keyexpr, sizeof(pub->topic_keyexpr), "%s", clean_topic);
  z_view_keyexpr_from_str_unchecked(&ke, pub->topic_keyexpr);

  z_publisher_options_t pub_opts;
  z_publisher_options_default(&pub_opts);

  pub_opts.reliability = Z_RELIABILITY_BEST_EFFORT;
  pub_opts.congestion_control = Z_CONGESTION_CONTROL_DROP;

  if (
    z_declare_publisher(
      z_session_loan(&z_session), &pub->z_pub, z_view_keyexpr_loan(&ke), &pub_opts) != Z_OK)
  {
    pub->in_use = false;
    return NULL;
  }
#endif

#if defined(CONFIG_ZENBEDDED_TIER_1)
  char mangled_topic[CONFIG_ZENBEDDED_MAX_TOPIC_NAME_LEN];
  mangle_topic(mangled_topic, sizeof(mangled_topic), clean_topic);

  pub->attachment.sequence_number = 0;
  pub->attachment.gid_length = RMW_GID_SIZE;
  sys_rand_get(pub->attachment.gid, RMW_GID_SIZE);

  z_id_t zid = z_info_zid(z_session_loan(&z_session));
  snprintf(
    pub->lv_keyexpr, sizeof(pub->lv_keyexpr),
    "@ros2_lv/%u/%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x/0/10/MP/%%/%%/%s/"
    "%%%s/%s/%s/::,7:,:,:,,",
    current_domain_id, zid.id[0], zid.id[1], zid.id[2], zid.id[3], zid.id[4], zid.id[5], zid.id[6],
    zid.id[7], zid.id[8], zid.id[9], zid.id[10], zid.id[11], zid.id[12], zid.id[13], zid.id[14],
    zid.id[15], current_node_name, mangled_topic, type_name, type_hash);

  z_view_keyexpr_t lv_ke;
  z_view_keyexpr_from_str(&lv_ke, pub->lv_keyexpr);
  z_liveliness_declare_token(
    z_session_loan(&z_session), &pub->lv_token, z_view_keyexpr_loan(&lv_ke), NULL);
#endif

  LOG_INF("Declared Publisher: %s", pub->topic_keyexpr);
  return pub;
}

int zenbedded_transport_publish(zenbedded_pub_t pub, const uint8_t * payload, size_t payload_size)
{
  if (!pub || !pub->in_use)
  {
    return -1;
  }

#if defined(CONFIG_ZENBEDDED_TIER_1)
  struct timespec tv;
  clock_gettime(CLOCK_REALTIME, &tv);
  pub->attachment.sequence_number++;
  pub->attachment.timestamp_ns = (int64_t)tv.tv_sec * 1000000000LL + tv.tv_nsec;

  z_owned_bytes_t z_attachment;
  z_bytes_from_static_buf(
    &z_attachment, (const uint8_t *)&pub->attachment, sizeof(rmw_zenoh_attachment_t));

  z_publisher_put_options_t options;
  z_publisher_put_options_default(&options);
  options.attachment = z_bytes_move(&z_attachment);

  z_owned_bytes_t z_payload;
  z_bytes_from_static_buf(&z_payload, payload, payload_size);
  return z_publisher_put(z_publisher_loan(&pub->z_pub), z_bytes_move(&z_payload), &options) == Z_OK
           ? 0
           : -1;

#elif defined(CONFIG_ZENBEDDED_TIER_2)
  z_publisher_put_options_t options;
  z_publisher_put_options_default(&options);

  z_owned_bytes_t z_payload;
  z_bytes_from_static_buf(&z_payload, payload, payload_size);
  return z_publisher_put(z_publisher_loan(&pub->z_pub), z_bytes_move(&z_payload), &options) == Z_OK
           ? 0
           : -1;
#endif

  return -1;
}

zenbedded_sub_t zenbedded_transport_declare_subscriber(
  const char * topic_name, const char * type_name, zenbedded_recv_cb_t callback, void * user_data)
{
  if (atomic_get(&transport_ref_count) <= 0)
  {
    return NULL;
  }

  struct zenbedded_sub_s * sub = NULL;
  for (int i = 0; i < MAX_SUBSCRIBERS; i++)
  {
    if (!sub_pool[i].in_use)
    {
      sub = &sub_pool[i];
      sub->in_use = true;
      break;
    }
  }
  if (!sub)
  {
    LOG_ERR("Subscriber pool exhausted! Max limit: %d", MAX_SUBSCRIBERS);
    return NULL;
  }

  sub->user_cb = callback;
  sub->user_data = user_data;

  const char * clean_topic = (topic_name[0] == '/') ? topic_name + 1 : topic_name;

#if defined(CONFIG_ZENBEDDED_TIER_1)
  if (type_name == NULL)
  {
    LOG_ERR("Subscriber declaration failed: type_name cannot be NULL in Tier 1");
    sub->in_use = false;
    return NULL;
  }
  const char * type_hash = zenbedded_get_rihs_hash(type_name);
  if (type_hash == NULL)
  {
    LOG_ERR(
      "Subscriber declaration failed: unknown message type '%s' (not in RIHS ledger)", type_name);
    sub->in_use = false;
    return NULL;
  }
  snprintf(
    sub->topic_keyexpr, sizeof(sub->topic_keyexpr), "%u/%s/%s/%s", current_domain_id, clean_topic,
    type_name, type_hash);
#elif defined(CONFIG_ZENBEDDED_TIER_2)
  snprintf(sub->topic_keyexpr, sizeof(sub->topic_keyexpr), "%s", clean_topic);
#endif

  z_view_keyexpr_t ke;
  z_view_keyexpr_from_str_unchecked(&ke, sub->topic_keyexpr);

  z_owned_closure_sample_t sub_cb;
  z_closure_sample(&sub_cb, internal_sub_handler, NULL, sub);

#if defined(CONFIG_ZENBEDDED_TIER_1)
  if (
    z_declare_subscriber(
      z_session_loan(&z_session), &sub->z_sub, z_view_keyexpr_loan(&ke),
      z_closure_sample_move(&sub_cb), NULL) != Z_OK)
  {
    sub->in_use = false;
    LOG_ERR("Failed to declare Zenoh subscriber for %s", sub->topic_keyexpr);
    return NULL;
  }
#elif defined(CONFIG_ZENBEDDED_TIER_2)
  if (
    z_declare_subscriber(
      z_session_loan(&z_session), &sub->z_sub, z_view_keyexpr_loan(&ke),
      z_closure_sample_move(&sub_cb), NULL) != Z_OK)
  {
    sub->in_use = false;
    LOG_ERR("Failed to declare Zenoh subscriber for %s", sub->topic_keyexpr);
    return NULL;
  }
#endif

#if defined(CONFIG_ZENBEDDED_TIER_1)
  char mangled_topic[CONFIG_ZENBEDDED_MAX_TOPIC_NAME_LEN];
  mangle_topic(mangled_topic, sizeof(mangled_topic), clean_topic);

  z_id_t zid = z_info_zid(z_session_loan(&z_session));
  snprintf(
    sub->lv_keyexpr, sizeof(sub->lv_keyexpr),
    "@ros2_lv/%u/%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x/0/10/MS/%%/%%/%s/"
    "%%%s/%s/%s/::,7:,:,:,,",
    current_domain_id, zid.id[0], zid.id[1], zid.id[2], zid.id[3], zid.id[4], zid.id[5], zid.id[6],
    zid.id[7], zid.id[8], zid.id[9], zid.id[10], zid.id[11], zid.id[12], zid.id[13], zid.id[14],
    zid.id[15], current_node_name, mangled_topic, type_name, type_hash);

  z_view_keyexpr_t lv_ke;
  z_view_keyexpr_from_str(&lv_ke, sub->lv_keyexpr);
  z_liveliness_declare_token(
    z_session_loan(&z_session), &sub->lv_token, z_view_keyexpr_loan(&lv_ke), NULL);
#endif

  LOG_INF("Declared Subscriber: %s", sub->topic_keyexpr);
  return sub;
}
