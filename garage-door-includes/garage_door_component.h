//inspiration: https://github.com/glmnet/esphome_devices/blob/master/my_home/arduino_port_expander.h
#include "esphome.h"

//#define LOG_DEBUG   true

#define SYNC_BYTE   0x55

#define garage_door_get_comp(constructor) static_cast<GarageDoorComponent *>(constructor.get_component(0))

//#define garage_door_cover(ape, pin) garage_door_get_comp(ape)->get_cover(pin)

typedef enum
{
  hoermann_state_stopped = 0,
  hoermann_state_open,
  hoermann_state_closed,
  hoermann_state_venting,
  hoermann_state_opening,
  hoermann_state_closing,
  hoermann_state_error,
  hoermann_state_unkown
} hoermann_state_t;

typedef enum
{
  hoermann_action_stop = 0,
  hoermann_action_open,
  hoermann_action_close,
  hoermann_action_venting,
  hoermann_action_toggle_light,
  hoermann_action_none
} hoermann_action_t;

class GarageDoorComponent;

class GarageDoorVentingSwitch : public Switch {
  public:
    GarageDoorVentingSwitch(GarageDoorComponent *parent) {
      this->parent_ = parent;
    };
    void write_state(bool state) override;
  private:
    GarageDoorComponent *parent_;

  friend class GarageDoorComponent;
};

class GarageDoorLightSwitch : public Switch {
  public:
    GarageDoorLightSwitch(GarageDoorComponent *parent) {
      this->parent_ = parent;
    };
    void write_state(bool state) override;
  private:
    GarageDoorComponent *parent_;

  friend class GarageDoorComponent;
};

class GarageDoorComponent : public Component, public UARTDevice {
  public:
    GarageDoorComponent(UARTComponent *parent) : UARTDevice(parent) {
      actual_state = hoermann_state_unkown;
      actual_state_string = "unkown";
#ifdef LOG_DEBUG
      logcounter = 0;
      logcounter2 = 0;
#endif
    }

    TextSensor *state_sensor = new TextSensor();
    GarageDoorVentingSwitch *venting_switch = new GarageDoorVentingSwitch(this);
    GarageDoorLightSwitch *light_switch = new GarageDoorLightSwitch(this);

    void setup() override {
      ESP_LOGCONFIG("garage", "garage setup called!");
    }

    void loop() override {
#ifdef LOG_DEBUG
      logcounter++;
      if (logcounter > 1000)
      {
        ESP_LOGW("garage", "garage loop %i", logcounter2);
        logcounter=0;
      }
#endif

      if (read_rs232() == true)
      {
#ifdef LOG_DEBUG
        logcounter2++;
#endif
        parse_input();
      }
      if (actual_action != hoermann_action_none)
      {
        send_command();
        actual_action = hoermann_action_none;
      }

      current_door_state = get_state();
      if (current_door_state != last_door_state)
      {
        String state = get_state_string();
        ESP_LOGW("garage", "garage State changed!");
        //ESP_LOGW("custom", state.c_str());
        state_sensor->publish_state(state.c_str());
        last_door_state = current_door_state;
      }

    }

    hoermann_state_t get_state(void)
    {
      return actual_state;
    }

    String get_state_string(void)
    {
      return actual_state_string;
    }

    void action_open()
    {
      ESP_LOGD("garage", "action_open called");
      actual_action = hoermann_action_open;
    }

    void action_close()
    {
      ESP_LOGD("garage", "action_close called");
      actual_action = hoermann_action_close;
    }

    void action_stop()
    {
      ESP_LOGD("garage", "action_stop called");
      actual_action = hoermann_action_stop;
    }

    void action_venting()
    {
      ESP_LOGD("garage", "action_venting called");
      actual_action = hoermann_action_venting;
    }

    void action_toggle_light()
    {
      ESP_LOGD("garage", "action_toggle_light called");
      actual_action = hoermann_action_toggle_light;
    }

  private:

    hoermann_state_t current_door_state = hoermann_state_unkown;
    hoermann_state_t last_door_state = hoermann_state_unkown;

    hoermann_state_t actual_state;
    String actual_state_string;
    hoermann_action_t actual_action;
    uint8_t rx_buffer[16];
    uint8_t output_buffer[16];

#ifdef LOG_DEBUG
    uint32_t logcounter;
    uint32_t logcounter2;
    String lastlog;
#endif

    bool read_rs232(void)
    {
      static uint8_t counter = 0;
      static uint8_t len = 0;
      uint8_t data;

      while (available() > 0)
      {
        // read the incoming byte:
        data = (uint8_t)read();

        if ((data == SYNC_BYTE) && (counter == 0))
        {
          rx_buffer[counter] = data;
          counter++;
          len = 0;
        }
        else if (counter > 0)
        {
          rx_buffer[counter] = data;
          counter++;
          if (counter == 3)
          {
            if (data < 16)
            {
              len = data + 4; //3 = SYNC + CMD + LEN + CHK, limit to 15 data bytes
            }
            else
            {
              counter = 0;
            }
          }
          else if (counter == len)
          {
            if (calc_checksum(rx_buffer, len - 1) == data)
            {
              counter = 0;
              return true;
            }
            counter = 0;
          }
        } else {
          ESP_LOGD("garage", "read_rs232, wrong SYNC byte data = %i", data);
        }
      }

      return false;
    }

    void parse_input(void)
    {
#ifdef LOG_DEBUG
      ESP_LOGD("garage", "parse_input rx_buffer[0] = %i", rx_buffer[0]);
      ESP_LOGD("garage", "parse_input rx_buffer[1] = %i", rx_buffer[1]);
      ESP_LOGD("garage", "parse_input rx_buffer[2] = %i", rx_buffer[2]);
      ESP_LOGD("garage", "parse_input rx_buffer[3] = %i", rx_buffer[3]);
      ESP_LOGD("garage", "parse_input rx_buffer[4] = %i", rx_buffer[4]);
      ESP_LOGD("garage", "parse_input rx_buffer[5] = %i", rx_buffer[5]);
      ESP_LOGD("garage", "parse_input rx_buffer[6] = %i", rx_buffer[6]);
#endif
      if (rx_buffer[1] == 0x00)
      {
        if (rx_buffer[2] == 0x02)
        {
#ifdef LOG_DEBUG
          ESP_LOGD("garage", "parse_input 3 = %i", rx_buffer[3]);
#endif
          if ((rx_buffer[3] & 0x01) == 0x01)
          {
            actual_state = hoermann_state_open;
            actual_state_string = "open";
          }
          else if ((rx_buffer[3] & 0x02) == 0x02)
          {
            actual_state = hoermann_state_closed;
            actual_state_string = "closed";
          }
          else if ((rx_buffer[3] & 0x80) == 0x80)
          {
            actual_state = hoermann_state_venting;
            actual_state_string = "venting";
          }
          else if ((rx_buffer[3] & 0x60) == 0x40)
          {
            actual_state = hoermann_state_opening;
            actual_state_string = "opening";
          }
          else if ((rx_buffer[3] & 0x60) == 0x60)
          {
            actual_state = hoermann_state_closing;
            actual_state_string = "closing";
          }
          else if ((rx_buffer[3] & 0x10) == 0x10)
          {
            actual_state = hoermann_state_error;
            actual_state_string = "error";
          }
          else
          {
            actual_state = hoermann_state_stopped;
            actual_state_string = "stopped";
          }
        }
      }
    }

    void send_command(void)
    {
      output_buffer[0] = 0x55;
      output_buffer[1] = 0x01;
      output_buffer[2] = 0x01;
      output_buffer[3] = (uint8_t)actual_action;
      output_buffer[4] = output_buffer[0] + output_buffer[1] + output_buffer[2] + output_buffer[3];
      write_array(&output_buffer[0], 5);
    }

    uint8_t calc_checksum(uint8_t *p_data, uint8_t length)
    {
      uint8_t i;
      uint8_t crc = 0;

      for (i = 0; i < length; i++)
      {
        crc += *p_data;
        p_data++;
      }

      return crc;
    }

};

void GarageDoorVentingSwitch::write_state(bool state)
{
  if (state)
  {
    this->parent_->action_venting();
  }
  else
  {
    this->parent_->action_close();
  }
  publish_state(state);
}

void GarageDoorLightSwitch::write_state(bool state)
{
  if (state)
  {
    this->parent_->action_toggle_light();
  }
  publish_state(state);
}
