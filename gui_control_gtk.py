import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gdk, GLib, Pango

import lgpio
import Adafruit_ADS1x15
import time
import threading
import os
from datetime import datetime
import subprocess
import gc

MOSFET_CHANNELS = [12, 13, 19, 16]
NUM_CHANNELS = len(MOSFET_CHANNELS)

try:
    LGPIO_HANDLE = lgpio.gpiochip_open(4)
    for pin in MOSFET_CHANNELS:
        lgpio.gpio_claim_output(LGPIO_HANDLE, pin)
except Exception as e:
    print(f"⚠️ lgpio 초기화 실패: {e}")
    LGPIO_HANDLE = -1

try:
    ads = Adafruit_ADS1x15.ADS1115(busnum=4)
    GAIN = 1
    print("✅ I2C 버스 4번으로 ADC 초기화 성공.")
except Exception as e:
    print(f"⚠️ I2C 버스 4번 ADC 초기화 실패: {e}")
    ads = None

i2c_lock = threading.Lock()

BASE_LOG_DIR = os.path.join(os.path.expanduser('~'), 'Desktop', '로그파일')
LOG_DIRECTORY = ""
log_file_path = ""
last_log_hour = -1

def setup_logging():
    global log_file_path, LOG_DIRECTORY, last_log_hour
    try:
        today_str = time.strftime("%Y-%m-%d")
        LOG_DIRECTORY = os.path.join(BASE_LOG_DIR, today_str)
        if not os.path.isdir(LOG_DIRECTORY):
            os.makedirs(LOG_DIRECTORY)
        current_hour = datetime.now().hour
        last_log_hour = current_hour
        timestamp = time.strftime("%Y-%m-%d_%H-00-00")
        log_file_path = os.path.join(LOG_DIRECTORY, f"{timestamp}_system.log")
        print(f"시스템 로그 파일이 '{log_file_path}'에 생성됩니다.")
    except Exception as e:
        print(f"로그 파일 설정 오류: {e}")

def check_and_rotate_log():
    global log_file_path, last_log_hour
    current_hour = datetime.now().hour
    is_midnight_rollover = (current_hour == 0 and last_log_hour == 23)
    is_forward_in_time = (current_hour > last_log_hour)

    if is_midnight_rollover or is_forward_in_time:
        today_str = time.strftime("%Y-%m-%d")
        LOG_DIRECTORY = os.path.join(BASE_LOG_DIR, today_str)
        if not os.path.isdir(LOG_DIRECTORY):
            os.makedirs(LOG_DIRECTORY)
        timestamp = time.strftime("%Y-%m-%d_%H-00-00")
        new_log_file_path = os.path.join(LOG_DIRECTORY, f"{timestamp}_system.log")
        log_file_path = new_log_file_path
        last_log_hour = current_hour
        log_message("INFO", f"새로운 시간대 로그 파일로 전환: {log_file_path}")

def log_message(level, message, data=None):
    check_and_rotate_log()
    if not log_file_path:
        return
    try:
        if level == "TX" and data is not None:
            output_hz_str = ", ".join([f"{hz:.1f}" for hz in data['output_hz_list']])
            timestamp = datetime.now().strftime('%Y%m%d %H:%M:%S')
            log_entry = f"{timestamp}, {data['input_hz']}, {data['pressure']:.1f}, {output_hz_str}\n"
        else:
            timestamp_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            log_entry = f"[{timestamp_str}] [{level}] {message}\n"
        with open(log_file_path, "a", encoding='utf-8') as f:
            f.write(log_entry)
    except Exception as e:
        print(f"로그 기록 오류: {e}")

ZERO_PRESSURE_VOLTAGE = 0.19825
VOLTAGE_PER_KPA = 0.03875 / 50

class FrequencyMonitor(threading.Thread):
    def __init__(self, adc_channel):
        super().__init__()
        self.daemon = True
        self.adc_channel = adc_channel
        self.running = True
        self.lock = threading.Lock()
        self._frequency = 0.0
        self.CURRENT_THRESHOLD = 0.05
        self.SAMPLING_INTERVAL = 0.0015

    def run(self):
        last_state_is_on = False
        last_rising_edge_time = time.time()
        while self.running:
            if ads is None:
                time.sleep(0.1)
                continue
            try:
                with i2c_lock:
                    raw_value = ads.read_adc(self.adc_channel, gain=GAIN)
                measured_voltage = raw_value * 4.096 / 32767.0
                
                pseudo_current = (measured_voltage - 1.25) / 0.185
                current_state_is_on = pseudo_current > self.CURRENT_THRESHOLD
                now = time.time()
                
                if current_state_is_on and not last_state_is_on:
                    period = now - last_rising_edge_time
                    last_rising_edge_time = now
                    if period > 0.001: 
                        freq = 1.0 / period
                        if 5 < freq < 50:
                            with self.lock:
                                self._frequency = freq
                
                if now - last_rising_edge_time > 0.5:
                    with self.lock:
                        self._frequency = 0.0
                
                last_state_is_on = current_state_is_on
                time.sleep(self.SAMPLING_INTERVAL)
            except Exception as e:
                time.sleep(0.1)

    def get_frequency(self):
        with self.lock:
            return self._frequency

    def stop(self):
        self.running = False

class ControlWindow(Gtk.Window):
    def __init__(self):
        super().__init__(title="Air Pump Controller")
        self.set_default_size(800, 480) # 5인치 해상도(예: 800x480)에 맞게 설정
        self.move(0, 0)
        self.set_decorated(False)
        self.fullscreen()

        self.run_thread = True
        self.motor_running = False
        self.active_freq = 10
        self.pending_freq = 10

        self.latest_pressure_kpa = 0.0
        self.output_frequencies = [0.0] * NUM_CHANNELS
        self.sensor_lock = threading.Lock()

        self.sensor_reader_thread = threading.Thread(target=self.high_speed_sensor_reader, daemon=True)
        self.sensor_reader_thread.start()

        self.freq_monitor_1 = FrequencyMonitor(1)
        self.freq_monitor_1.start()
        
        # --- 레이아웃 간격 축소 ---
        main_grid = Gtk.Grid(column_spacing=10, row_spacing=10, margin=10)
        # ------------------------
        main_grid.set_hexpand(True) 
        main_grid.set_vexpand(True) 
        main_grid.set_column_homogeneous(True)
        self.add(main_grid)

        self.create_display_area(main_grid)
        self.create_control_area(main_grid)
        self.create_status_area(main_grid)
        self.create_system_buttons(main_grid)

        self.apply_styles()
        
        GLib.timeout_add(100, self.update_gui)

    def create_display_area(self, grid):
        self.display_labels = {}
        
        # --- 고정 높이 제거 또는 축소 ---
        # square_height = 150 # 고정 높이가 필요하다면 값을 줄임
        # ---------------------------

        self.display_labels['pressure_val'] = self.create_display_box("공급 압력", "0.0 kPa")
        # self.display_labels['pressure_val'].set_size_request(-1, square_height) # 고정 높이 제거
        grid.attach(self.display_labels['pressure_val'], 0, 0, 2, 1)

        self.display_labels['pulse_in_val'] = self.create_display_box("Pulse In", f"{self.pending_freq} Hz")
        # self.display_labels['pulse_in_val'].set_size_request(-1, square_height) # 고정 높이 제거
        grid.attach(self.display_labels['pulse_in_val'], 2, 0, 2, 1)

        self.display_labels['pulse_out_vals'] = []
        for i in range(NUM_CHANNELS):
            label = self.create_display_box(f"Pulse Out {i+1}", "0.0 Hz")
            # label.set_size_request(-1, square_height) # 고정 높이 제거
            self.display_labels['pulse_out_vals'].append(label)
            grid.attach(label, 4 + i, 0, 1, 1)
            
    def create_display_box(self, title_text, value_text):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        box.set_name("display_box")
        title = Gtk.Label(label=title_text)
        title.set_name("display_title")
        value = Gtk.Label(label=value_text)
        value.set_name("display_value")
        box.pack_start(title, False, False, 0)
        box.pack_start(value, True, True, 0)
        return box

    def create_control_area(self, grid):
        control_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10, halign=Gtk.Align.CENTER)
        control_box.set_vexpand(True)
        control_box.set_valign(Gtk.Align.CENTER)
        grid.attach(control_box, 0, 1, 4 + NUM_CHANNELS, 1)

        up_button = Gtk.Button(label="▲")
        up_button.connect("clicked", self.on_up_down_clicked, 1)
        up_button.set_name("control_button_arrow")
        
        down_button = Gtk.Button(label="▼")
        down_button.connect("clicked", self.on_up_down_clicked, -1)
        down_button.set_name("control_button_arrow")

        set_button = Gtk.Button(label="SET")
        set_button.connect("clicked", self.on_set_clicked)
        set_button.set_name("control_button_set")

        stop_button = Gtk.Button(label="STOP")
        stop_button.connect("clicked", self.on_stop_clicked)
        stop_button.set_name("control_button_stop")

        up_button.set_hexpand(True)
        down_button.set_hexpand(True)
        set_button.set_hexpand(True)
        stop_button.set_hexpand(True)

        control_box.pack_start(up_button, True, True, 0)
        control_box.pack_start(down_button, True, True, 0)
        control_box.pack_start(set_button, True, True, 0)
        control_box.pack_start(stop_button, True, True, 0)

    def create_status_area(self, grid):
        self.channel_status_labels = []
        status_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=0, halign=Gtk.Align.CENTER)
        status_box.set_name("status_bar")
        grid.attach(status_box, 0, 2, 4 + NUM_CHANNELS, 1)
        
        pressure_status = Gtk.Label(label="압력")
        pressure_status.set_name("status_item")
        status_box.pack_start(pressure_status, True, True, 0)

        for i in range(NUM_CHANNELS):
            label = Gtk.Label(label=f"{i+1}")
            label.set_name("status_item")
            status_box.pack_start(label, True, True, 0)
            self.channel_status_labels.append(label)

    def create_system_buttons(self, grid):
        bottom_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, margin_top=5)
        bottom_box.set_valign(Gtk.Align.END)
        grid.attach(bottom_box, 0, 3, 4 + NUM_CHANNELS, 1)
        
        log_dir_button = Gtk.Button(label="로그파일")
        log_dir_button.connect("clicked", lambda w: subprocess.Popen(['xdg-open', BASE_LOG_DIR]))
        
        shutdown_button = Gtk.Button(label="시스템 종료")
        shutdown_button.connect("clicked", lambda w: os.system("sudo shutdown -h now"))
        
        bottom_box.pack_end(shutdown_button, False, False, 5)
        bottom_box.pack_end(log_dir_button, False, False, 5)

    def apply_styles(self):
        css_provider = Gtk.CssProvider()
        # --- 5인치 화면에 맞게 CSS 수정 ---
        css_data = b"""
            window { background-color: #333; color: white; }
            #display_box { 
                background-color: #111; 
                border: 1px solid #555;
                border-radius: 6px;
                padding: 3px;
            }
            #display_title { 
                font-size: 12pt; 
                font-weight: bold; 
                color: #AAA;
            }
            #display_value { 
                font-size: 22pt; 
                font-weight: bold; 
                color: white;
            }
            #control_button_arrow { font-size: 24pt; min-height: 60px; }
            #control_button_set { font-size: 20pt; font-weight: bold; background-color: #2196F3; color: white; min-height: 60px; }
            #control_button_stop { font-size: 20pt; font-weight: bold; background-color: #F44336; color: white; min-height: 60px; }
            #status_bar { background-color: #000; border-radius: 5px; }
            #status_item { 
                font-size: 16pt; 
                font-weight: bold; 
                padding: 5px;
                min-width: 70px;
            }
            .connected { color: #4CAF50; }
            .disconnected { color: #F44336; }
            button { border-radius: 6px; }
        """
        # ---------------------------------
        css_provider.load_from_data(css_data)
        Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), css_provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

    def on_up_down_clicked(self, button, step):
        self.pending_freq = max(5, min(40, self.pending_freq + step))
        self.display_labels['pulse_in_val'].get_children()[1].set_text(f"{self.pending_freq} Hz")
        log_message("INFO", f"주파수 조정 시도: {self.pending_freq} Hz")

    def on_set_clicked(self, button):
        self.active_freq = self.pending_freq
        self.motor_running = True
        
        if LGPIO_HANDLE >= 0:
            for pin in MOSFET_CHANNELS:
                lgpio.tx_pwm(LGPIO_HANDLE, pin, self.active_freq, 50)
            
        status_text = f"펌프 시작/주파수 설정: {self.active_freq} Hz"
        log_message("INFO", status_text)

    def on_stop_clicked(self, button):
        if self.motor_running:
            self.motor_running = False
            if LGPIO_HANDLE >= 0:
                for pin in MOSFET_CHANNELS:
                    lgpio.tx_pwm(LGPIO_HANDLE, pin, self.active_freq, 0)
            log_message("INFO", "모든 펌프 정지")

    def high_speed_sensor_reader(self):
        gc.disable() 
        while self.run_thread:
            if ads is None:
                time.sleep(0.1)
                continue
            pressure = self.read_pressure_sensor()
            output_hz_1 = self.freq_monitor_1.get_frequency()
            
            if pressure is not None:
                with self.sensor_lock:
                    self.latest_pressure_kpa = pressure
                    self.output_frequencies[0] = output_hz_1
                    
            time.sleep(0.002)

    def read_pressure_sensor(self):
        try:
            with i2c_lock:
                raw_value = ads.read_adc(0, gain=GAIN)
            measured_voltage = raw_value * 4.096 / 32767.0
            pressure_kpa = (measured_voltage - ZERO_PRESSURE_VOLTAGE) / VOLTAGE_PER_KPA
            return max(0.0, pressure_kpa)
        except Exception:
            return None

    def update_gui(self):
        if not self.run_thread:
            return False

        with self.sensor_lock:
            current_pressure = self.latest_pressure_kpa
            current_output_freqs = list(self.output_frequencies)

        self.display_labels['pressure_val'].get_children()[1].set_text(f"{current_pressure:.1f} kPa")
        self.display_labels['pulse_in_val'].get_children()[1].set_text(f"{self.pending_freq} Hz")
        
        for i in range(NUM_CHANNELS):
            if i < len(current_output_freqs):
                self.display_labels['pulse_out_vals'][i].get_children()[1].set_text(f"{current_output_freqs[i]:.1f}")
            else:
                 self.display_labels['pulse_out_vals'][i].get_children()[1].set_text("0.0")
        
        self.update_connection_status(current_output_freqs)
        
        return True

    def update_connection_status(self, current_output_freqs):
        is_connected_1 = not (self.motor_running and current_output_freqs[0] < 1.0)
        self.set_label_connection_style(self.channel_status_labels[0], is_connected_1)
        
        for i in range(1, NUM_CHANNELS):
            self.set_label_connection_style(self.channel_status_labels[i], False)

    def set_label_connection_style(self, label, is_connected):
        context = label.get_style_context()
        if is_connected:
            context.remove_class("disconnected")
            context.add_class("connected")
        else:
            context.remove_class("connected")
            context.add_class("disconnected")

    def on_destroy(self, *args):
        log_message("INFO", "GUI 창이 닫혔습니다. 리소스 해제 중...")
        self.run_thread = False
        self.freq_monitor_1.stop()
        
        time.sleep(0.2)
        if LGPIO_HANDLE >= 0:
            for pin in MOSFET_CHANNELS:
                lgpio.tx_pwm(LGPIO_HANDLE, pin, 10, 0)
            lgpio.gpiochip_close(LGPIO_HANDLE)
        
        Gtk.main_quit()

if __name__ == "__main__":
    try:
        os.nice(-20)
    except Exception as e:
        print(f"프로세스 우선순위 설정 오류: {e}")

    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    setup_logging()
    win = ControlWindow()
    win.connect("destroy", win.on_destroy)
    win.show_all()
    Gtk.main()
