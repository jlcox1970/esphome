#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/network/ip_address.h"

#ifdef USE_ESP32

#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_netif.h"

namespace esphome {
namespace ethernet {

enum EthernetType {
  UNKNOWN = 0,
  ETHERNET_TYPE_LAN8720,
  ETHERNET_TYPE_RTL8201,
  ETHERNET_TYPE_DP83848,
  ETHERNET_TYPE_IP101,
  ETHERNET_TYPE_JL1101,
  ETHERNET_TYPE_KSZ8081,
};

struct ManualIP {
  network::IPAddress static_ip;
  network::IPAddress gateway;
  network::IPAddress subnet;
  network::IPAddress dns1;  ///< The first DNS server. 0.0.0.0 for default.
  network::IPAddress dns2;  ///< The second DNS server. 0.0.0.0 for default.
};

enum class EthernetComponentState {
  STOPPED,
  CONNECTING,
  CONNECTED,
};

typedef struct {
    esp_eth_mediator_t mediator;
    esp_eth_phy_t *phy;
    esp_eth_mac_t *mac;
    esp_timer_handle_t check_link_timer;
    uint32_t check_link_period_ms;
    eth_speed_t speed;
    eth_duplex_t duplex;
    eth_link_t link;
} esp_eth_driver_t;

typedef struct {
    esp_eth_phy_t parent;
    esp_eth_mediator_t *eth;
    int addr;
    uint32_t reset_timeout_ms;
    uint32_t autonego_timeout_ms;
    eth_link_t link_status;
    int reset_gpio_num;
    int vendor_model;
} phy_ksz80xx_t;

class EthernetComponent : public Component {
 public:
  EthernetComponent();
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;
  bool can_proceed() override;
  bool is_connected();

  void set_phy_addr(uint8_t phy_addr);
  void set_power_pin(int power_pin);
  void set_mdc_pin(uint8_t mdc_pin);
  void set_mdio_pin(uint8_t mdio_pin);
  void set_type(EthernetType type);
  void set_clk_mode(emac_rmii_clock_mode_t clk_mode, emac_rmii_clock_gpio_t clk_gpio);
  void set_manual_ip(const ManualIP &manual_ip);

  network::IPAddress get_ip_address();
  std::string get_use_address() const;
  void set_use_address(const std::string &use_address);

 protected:
  static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
  static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

  void start_connect_();
  void dump_connect_params_();
  void dump_phy_registers_();
  void set_phy_registers_();

  std::string use_address_;
  uint8_t phy_addr_{0};
  int power_pin_{-1};
  uint8_t mdc_pin_{23};
  uint8_t mdio_pin_{18};
  EthernetType type_{ETHERNET_TYPE_LAN8720};
  emac_rmii_clock_mode_t clk_mode_{EMAC_CLK_EXT_IN};
  emac_rmii_clock_gpio_t clk_gpio_{EMAC_CLK_IN_GPIO};
  optional<ManualIP> manual_ip_{};

  bool started_{false};
  bool connected_{false};
  EthernetComponentState state_{EthernetComponentState::STOPPED};
  uint32_t connect_begin_;
  esp_netif_t *eth_netif_{nullptr};
  esp_eth_handle_t eth_handle_{nullptr};
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern EthernetComponent *global_eth_component;
extern "C" esp_eth_phy_t *esp_eth_phy_new_jl1101(const eth_phy_config_t *config);

}  // namespace ethernet
}  // namespace esphome

#endif  // USE_ESP32
