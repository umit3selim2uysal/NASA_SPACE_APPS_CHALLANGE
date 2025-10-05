# /// Gorsellestirme_pyn
# requires-python = ">=3.7"
# dependencies = [
#   "PyQt5>=5.15.10",
#   "pyserial>=3.5",
#   "pyqtgraph>=0.13.3",
#   "numpy>=1.24.3",
#   "folium>=0.14.0",
#   "PyQtWebEngine>=5.15.0"
# ]
# ///

"""
Seri Port Veri Görselleştirici
PyQt5 tabanlı şık arayüz ile seri port verilerini görüntüleme uygulaması
"""

import sys
import time
import json
import tempfile
import os
from datetime import datetime
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QPushButton, QComboBox, QLabel, QTextEdit, 
                             QLineEdit, QSpinBox, QGroupBox, QGridLayout, QSplitter,
                             QFrame, QScrollArea, QCheckBox, QProgressBar, QStatusBar,
                             QMessageBox, QTabWidget, QTableWidget, QTableWidgetItem)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt, QUrl
from PyQt5.QtGui import QFont, QColor, QPalette, QIcon
from PyQt5.QtWebEngineWidgets import QWebEngineView
import pyqtgraph as pg
import numpy as np
import folium


class SerialReaderThread(QThread):
    """Seri port okuma işlemini ayrı thread'de çalıştıran sınıf"""
    data_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = False
        
    def setup_serial(self, port, baudrate, timeout=1):
        """Seri port ayarlarını yapılandır"""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            return True
        except Exception as e:
            self.error_occurred.emit(f"Seri port hatası: {str(e)}")
            return False
    
    def run(self):
        """Thread ana döngüsü"""
        self.running = True
        while self.running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                        if data:
                            self.data_received.emit(data)
                except Exception as e:
                    self.error_occurred.emit(f"Veri okuma hatası: {str(e)}")
            else:
                self.msleep(100)
    
    def stop(self):
        """Thread'i durdur"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()


class SensorDataParser:
    """Sensor verilerini parse eden sınıf"""
    
    def __init__(self):
        self.reset_data()
        
    def reset_data(self):
        """Veri yapısını sıfırla"""
        self.current_data = {
            'timestamp': 0,
            'mq4_analog': 0,
            'mq4_voltage': 0.0,
            'mq4_ratio': 0.0,
            'mq4_digital': 0,
            'metan': 0.0,
            'dogalgaz': 0.0,
            'lpg': 0.0,
            'alkol': 0.0,
            'hidrojen': 0.0,
            'smoke': 0.0,
            'temperature': 0.0,
            'pressure': 0.0,
            'altitude': 0.0,
            'gps_latitude': 0.0,
            'gps_longitude': 0.0,
            'gps_altitude': 0.0,
            'gps_speed': 0.0,
            'gps_satellites': 0,
            'gps_fix_type': 0,
            'gas_detected': False
        }
        
    def parse_line(self, line):
        """Tek bir satırı parse et - hem eski format hem de yeni JSON format destekler"""
        line = line.strip()
        
        # JSON formatını kontrol et
        if line.startswith('{') and line.endswith('}'):
            try:
                json_data = json.loads(line)
                return self.parse_json_data(json_data)
            except json.JSONDecodeError:
                pass  # JSON parse edilemezse eski formata devam et
        
        # Eski format parsing (geriye uyumluluk için)
        if 'ZAMAN:' in line:
            try:
                self.current_data['timestamp'] = int(line.split(':')[1].strip())
            except:
                pass
                
        elif 'MQ4 Analog:' in line:
            try:
                self.current_data['mq4_analog'] = int(line.split(':')[1].strip())
            except:
                pass
                
        elif 'MQ4 Voltaj:' in line:
            try:
                voltaj_str = line.split(':')[1].strip().replace(' V', '')
                self.current_data['mq4_voltage'] = float(voltaj_str)
            except:
                pass
                
        elif 'MQ4 Oran:' in line:
            try:
                self.current_data['mq4_ratio'] = float(line.split(':')[1].strip())
            except:
                pass
                
        elif 'MQ4 Dijital:' in line:
            try:
                self.current_data['mq4_digital'] = int(line.split(':')[1].strip())
            except:
                pass
                
        elif 'Metan:' in line:
            try:
                self.current_data['metan'] = float(line.split(':')[1].strip())
            except:
                pass
                
        elif 'Dogalgaz:' in line:
            try:
                self.current_data['dogalgaz'] = float(line.split(':')[1].strip())
            except:
                pass
                
        elif 'LPG:' in line:
            try:
                self.current_data['lpg'] = float(line.split(':')[1].strip())
            except:
                pass
                
        elif 'Alkol:' in line:
            try:
                self.current_data['alkol'] = float(line.split(':')[1].strip())
            except:
                pass
                
        elif 'Hidrojen:' in line:
            try:
                self.current_data['hidrojen'] = float(line.split(':')[1].strip())
            except:
                pass
                
        elif 'Smoke:' in line:
            try:
                self.current_data['smoke'] = float(line.split(':')[1].strip())
            except:
                pass
                
        elif 'Sıcaklık:' in line:
            try:
                sicaklik_str = line.split(':')[1].strip().replace(' °C', '')
                self.current_data['temperature'] = float(sicaklik_str)
            except:
                pass
                
        elif 'Basınç:' in line:
            try:
                basinc_str = line.split(':')[1].strip().replace(' hPa', '')
                self.current_data['pressure'] = float(basinc_str)
            except:
                pass
                
        elif 'İrtifa:' in line:
            try:
                irtifa_str = line.split(':')[1].strip().replace(' m', '')
                self.current_data['altitude'] = float(irtifa_str)
            except:
                pass
                
        elif 'GPS:' in line:
            try:
                gps_data = line.split(':')[1].strip()
                lat_lon = gps_data.split(',')
                if len(lat_lon) == 2:
                    self.current_data['gps_latitude'] = float(lat_lon[0].strip())
                    self.current_data['gps_longitude'] = float(lat_lon[1].strip())
            except:
                pass
                
        elif 'GPS Yükseklik:' in line:
            try:
                yukseklik_str = line.split(':')[1].strip().replace(' m', '')
                if yukseklik_str != 'ovf':
                    self.current_data['gps_altitude'] = float(yukseklik_str)
            except:
                pass
                
        elif 'GPS Hız:' in line:
            try:
                hiz_str = line.split(':')[1].strip().replace(' km/h', '')
                if hiz_str != 'ovf':
                    self.current_data['gps_speed'] = float(hiz_str)
            except:
                pass
                
        elif 'Uydu Sayısı:' in line:
            try:
                self.current_data['gps_satellites'] = int(line.split(':')[1].strip())
            except:
                pass
                
        elif 'GPS Fix Tipi:' in line:
            try:
                self.current_data['gps_fix_type'] = int(line.split(':')[1].strip())
            except:
                pass
        
        return self.current_data.copy()
    
    def parse_json_data(self, json_data):
        """JSON formatındaki veriyi parse et"""
        try:
            # Ana JSON alanlarını al
            self.current_data['timestamp'] = json_data.get('timestamp', 0)
            self.current_data['mq4_analog'] = json_data.get('mq4_analog', 0)
            self.current_data['mq4_voltage'] = json_data.get('mq4_voltage', 0.0)
            self.current_data['mq4_ratio'] = json_data.get('mq4_ratio', 0.0)
            self.current_data['mq4_digital'] = json_data.get('mq4_digital', 0)
            
            # Gas concentrations object'ini parse et
            gas_concentrations = json_data.get('gas_concentrations', {})
            self.current_data['metan'] = gas_concentrations.get('Metan', 0.0)
            self.current_data['dogalgaz'] = gas_concentrations.get('Dogalgaz', 0.0)
            self.current_data['lpg'] = gas_concentrations.get('LPG', 0.0)
            self.current_data['alkol'] = gas_concentrations.get('Alkol', 0.0)
            self.current_data['hidrojen'] = gas_concentrations.get('Hidrojen', 0.0)
            self.current_data['smoke'] = gas_concentrations.get('Smoke', 0.0)
            
            # Çevre sensörleri
            self.current_data['temperature'] = json_data.get('temperature', 0.0)
            self.current_data['pressure'] = json_data.get('pressure', 0.0)
            self.current_data['altitude'] = json_data.get('altitude', 0.0)
            
            # GPS verileri
            self.current_data['gps_latitude'] = json_data.get('gps_latitude', 0.0)
            self.current_data['gps_longitude'] = json_data.get('gps_longitude', 0.0)
            self.current_data['gps_altitude'] = json_data.get('gps_altitude', 0.0)
            self.current_data['gps_speed'] = json_data.get('gps_speed', 0.0)
            self.current_data['gps_satellites'] = json_data.get('gps_satellites', 0)
            self.current_data['gps_fix_type'] = json_data.get('gps_fix_type', 0)
            
            # Gaz tespit durumu
            self.current_data['gas_detected'] = json_data.get('gas_detected', False)
            
        except Exception as e:
            print(f"JSON parse hatası: {e}")
        
        return self.current_data.copy()


class MapVisualization(QWidget):
    """GPS koordinatlarını harita üzerinde gösteren widget"""
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.gps_history = []
        self.map_file = None
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Harita kontrollleri
        control_layout = QHBoxLayout()
        
        self.coords_label = QLabel("GPS: 0.000000, 0.000000")
        self.coords_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        control_layout.addWidget(self.coords_label)
        
        self.clear_history_btn = QPushButton("Geçmişi Temizle")
        self.clear_history_btn.clicked.connect(self.clear_gps_history)
        control_layout.addWidget(self.clear_history_btn)
        
        self.center_map_btn = QPushButton("Mevcut Konuma Git")
        self.center_map_btn.clicked.connect(self.center_to_current_location)
        control_layout.addWidget(self.center_map_btn)
        
        control_layout.addStretch()
        layout.addLayout(control_layout)
        
        # Web view for map
        self.web_view = QWebEngineView()
        layout.addWidget(self.web_view)
        
        self.setLayout(layout)
        
        # İlk haritayı oluştur
        self.create_initial_map()
        
    def create_initial_map(self):
        """İlk haritayı oluştur"""
        # Varsayılan konum (İstanbul)
        initial_lat, initial_lon = 41.0082, 28.9784
        
        # Folium haritası oluştur
        m = folium.Map(
            location=[initial_lat, initial_lon],
            zoom_start=10,
            tiles='OpenStreetMap'
        )
        
        # İlk marker ekle
        folium.Marker(
            [initial_lat, initial_lon],
            popup="Başlangıç Konumu",
            icon=folium.Icon(color='red', icon='info-sign')
        ).add_to(m)
        
        # Geçici dosyaya kaydet
        self.map_file = tempfile.NamedTemporaryFile(mode='w', suffix='.html', delete=False)
        m.save(self.map_file.name)
        self.map_file.close()
        
        # Web view'a yükle
        self.web_view.load(QUrl.fromLocalFile(self.map_file.name))
        
    def update_location(self, lat, lon, altitude=0, speed=0, satellites=0):
        """GPS konumunu güncelle"""
        if lat == 0 and lon == 0:
            return  # Geçersiz koordinat
            
        self.current_lat = lat
        self.current_lon = lon
        
        # Koordinatları göster
        self.coords_label.setText(f"GPS: {lat:.6f}, {lon:.6f} | Alt: {altitude:.1f}m | Hız: {speed:.1f}km/h | Uydu: {satellites}")
        
        # GPS geçmişine ekle
        self.gps_history.append({
            'lat': lat,
            'lon': lon,
            'altitude': altitude,
            'speed': speed,
            'timestamp': datetime.now()
        })
        
        # Geçmişi sınırla (son 100 nokta)
        if len(self.gps_history) > 100:
            self.gps_history.pop(0)
            
        # Haritayı güncelle
        self.update_map()
        
    def update_map(self):
        """Haritayı güncel verilerle yeniden oluştur"""
        if not self.gps_history:
            return
            
        # Son konum
        last_pos = self.gps_history[-1]
        
        # Folium haritası oluştur
        m = folium.Map(
            location=[last_pos['lat'], last_pos['lon']],
            zoom_start=15,
            tiles='OpenStreetMap'
        )
        
        # Geçmiş rota çizgisi
        if len(self.gps_history) > 1:
            coordinates = [[pos['lat'], pos['lon']] for pos in self.gps_history]
            folium.PolyLine(
                coordinates,
                color='blue',
                weight=3,
                opacity=0.7,
                popup='GPS Rotası'
            ).add_to(m)
        
        # Başlangıç noktası (yeşil)
        if self.gps_history:
            first_pos = self.gps_history[0]
            folium.Marker(
                [first_pos['lat'], first_pos['lon']],
                popup=f"Başlangıç: {first_pos['timestamp'].strftime('%H:%M:%S')}",
                icon=folium.Icon(color='green', icon='play')
            ).add_to(m)
        
        # Mevcut konum (kırmızı)
        folium.Marker(
            [last_pos['lat'], last_pos['lon']],
            popup=f"Mevcut Konum\nYükseklik: {last_pos['altitude']:.1f}m\nHız: {last_pos['speed']:.1f}km/h\nZaman: {last_pos['timestamp'].strftime('%H:%M:%S')}",
            icon=folium.Icon(color='red', icon='record')
        ).add_to(m)
        
        # Ara noktalar (mavi)
        for i, pos in enumerate(self.gps_history[1:-1], 1):
            if i % 5 == 0:  # Her 5. noktayı göster
                folium.CircleMarker(
                    [pos['lat'], pos['lon']],
                    radius=3,
                    popup=f"Nokta {i}: {pos['timestamp'].strftime('%H:%M:%S')}",
                    color='blue',
                    fill=True
                ).add_to(m)
        
        # Geçici dosyayı güncelle
        if self.map_file:
            try:
                os.unlink(self.map_file.name)
            except:
                pass
                
        self.map_file = tempfile.NamedTemporaryFile(mode='w', suffix='.html', delete=False)
        m.save(self.map_file.name)
        self.map_file.close()
        
        # Web view'ı güncelle
        self.web_view.load(QUrl.fromLocalFile(self.map_file.name))
        
    def clear_gps_history(self):
        """GPS geçmişini temizle"""
        self.gps_history.clear()
        self.create_initial_map()
        
    def center_to_current_location(self):
        """Haritayı mevcut konuma odakla"""
        if self.current_lat != 0 or self.current_lon != 0:
            self.update_map()


class MultiChannelDataVisualization(QWidget):
    """Çoklu kanal veri görselleştirme widget'ı"""
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.data_buffers = {
            'mq4_analog': {'data': [], 'time': [], 'color': 'r'},
            'temperature': {'data': [], 'time': [], 'color': 'b'},
            'pressure': {'data': [], 'time': [], 'color': 'g'},
            'altitude': {'data': [], 'time': [], 'color': 'm'},
            'metan': {'data': [], 'time': [], 'color': 'c'},
            'lpg': {'data': [], 'time': [], 'color': 'y'},
            'smoke': {'data': [], 'time': [], 'color': 'orange'}
        }
        self.max_points = 500
        self.start_time = time.time()
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Ana grafik widget'ı
        self.main_plot = pg.PlotWidget()
        self.main_plot.setLabel('left', 'MQ4 Analog Değer')
        self.main_plot.setLabel('bottom', 'Zaman (s)')
        self.main_plot.setTitle('MQ4 Gaz Sensörü')
        self.main_plot.showGrid(x=True, y=True)
        self.main_plot.setBackground('w')
        self.main_plot.addLegend()
        
        # Sıcaklık ve basınç grafiği
        self.env_plot = pg.PlotWidget()
        self.env_plot.setLabel('left', 'Sıcaklık (°C) / Basınç (hPa/10)')
        self.env_plot.setLabel('bottom', 'Zaman (s)')
        self.env_plot.setTitle('Çevre Koşulları')
        self.env_plot.showGrid(x=True, y=True)
        self.env_plot.setBackground('w')
        self.env_plot.addLegend()
        
        # Gaz konsantrasyon grafiği
        self.gas_plot = pg.PlotWidget()
        self.gas_plot.setLabel('left', 'Konsantrasyon (ppm)')
        self.gas_plot.setLabel('bottom', 'Zaman (s)')
        self.gas_plot.setTitle('Gaz Konsantrasyonları')
        self.gas_plot.showGrid(x=True, y=True)
        self.gas_plot.setBackground('w')
        self.gas_plot.addLegend()
        
        # Grafik eğrilerini oluştur
        self.curves = {}
        self.curves['mq4_analog'] = self.main_plot.plot(pen=pg.mkPen(color='r', width=2), name='MQ4 Analog')
        self.curves['temperature'] = self.env_plot.plot(pen=pg.mkPen(color='b', width=2), name='Sıcaklık')
        self.curves['pressure'] = self.env_plot.plot(pen=pg.mkPen(color='g', width=2), name='Basınç/10')
        self.curves['metan'] = self.gas_plot.plot(pen=pg.mkPen(color='r', width=2), name='Metan')
        self.curves['lpg'] = self.gas_plot.plot(pen=pg.mkPen(color='b', width=2), name='LPG')
        self.curves['smoke'] = self.gas_plot.plot(pen=pg.mkPen(color='orange', width=2), name='Smoke')
        
        layout.addWidget(self.main_plot)
        layout.addWidget(self.env_plot)
        layout.addWidget(self.gas_plot)
        self.setLayout(layout)
        
    def add_sensor_data(self, sensor_data):
        """Sensor verilerini grafiğe ekle"""
        current_time = time.time() - self.start_time
        
        # MQ4 verileri
        if sensor_data.get('mq4_analog', 0) > 0:
            self.add_data_point('mq4_analog', sensor_data['mq4_analog'], current_time)
            
        # Çevre verileri
        if sensor_data.get('temperature', 0) > 0:
            self.add_data_point('temperature', sensor_data['temperature'], current_time)
            
        if sensor_data.get('pressure', 0) > 0:
            # Basıncı 10'a bölerek sıcaklık ile aynı ölçekte göster
            self.add_data_point('pressure', sensor_data['pressure'] / 10, current_time)
            
        # Gaz konsantrasyonları
        for gas_type in ['metan', 'lpg', 'smoke']:
            if sensor_data.get(gas_type, 0) > 0:
                self.add_data_point(gas_type, sensor_data[gas_type], current_time)
    
    def add_data_point(self, channel, value, timestamp):
        """Belirli bir kanala veri noktası ekle"""
        if channel in self.data_buffers:
            buffer = self.data_buffers[channel]
            buffer['data'].append(value)
            buffer['time'].append(timestamp)
            
            # Buffer boyutunu sınırla
            if len(buffer['data']) > self.max_points:
                buffer['data'].pop(0)
                buffer['time'].pop(0)
            
            # Grafiği güncelle
            if channel in self.curves:
                self.curves[channel].setData(buffer['time'], buffer['data'])
    
    def clear_data(self):
        """Tüm veri buffer'larını temizle"""
        for channel in self.data_buffers:
            self.data_buffers[channel]['data'].clear()
            self.data_buffers[channel]['time'].clear()
            if channel in self.curves:
                self.curves[channel].setData([], [])
        self.start_time = time.time()


class SerialMonitorApp(QMainWindow):
    """Ana uygulama penceresi"""
    
    def __init__(self):
        super().__init__()
        self.serial_thread = SerialReaderThread()
        self.parser = SensorDataParser()
        self.start_time = time.time()
        self.received_data_count = 0
        self.current_sensor_data = {}
        self.init_ui()
        self.setup_connections()
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_status)
        self.update_timer.start(1000)  # Her saniye güncelle
        
    def init_ui(self):
        """Kullanıcı arayüzünü oluştur"""
        self.setWindowTitle("Seri Port Veri İzleyici - Şık Arayüz")
        self.setGeometry(100, 100, 1200, 800)
        
        # Ana widget ve layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Sol panel - Kontroller
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, 1)
        
        # Sağ panel - Veri görüntüleme
        right_panel = self.create_data_panel()
        main_layout.addWidget(right_panel, 3)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Hazır")
        
        # Stil uygula
        self.apply_modern_style()
        
    def create_control_panel(self):
        """Kontrol panelini oluştur"""
        panel = QWidget()
        panel.setFixedWidth(300)
        layout = QVBoxLayout(panel)
        
        # Seri Port Ayarları
        port_group = QGroupBox("Seri Port Ayarları")
        port_layout = QGridLayout(port_group)
        
        # Port seçimi
        port_layout.addWidget(QLabel("Port:"), 0, 0)
        self.port_combo = QComboBox()
        self.refresh_ports()
        port_layout.addWidget(self.port_combo, 0, 1)
        
        self.refresh_btn = QPushButton("Yenile")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(self.refresh_btn, 0, 2)
        
        # Baud rate
        port_layout.addWidget(QLabel("Baud Rate:"), 1, 0)
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(['9600', '19200', '38400', '57600', '115200', '230400'])
        self.baudrate_combo.setCurrentText('115200')
        port_layout.addWidget(self.baudrate_combo, 1, 1, 1, 2)
        
        layout.addWidget(port_group)
        
        # Bağlantı Kontrolü
        connection_group = QGroupBox("Bağlantı")
        connection_layout = QVBoxLayout(connection_group)
        
        self.connect_btn = QPushButton("Bağlan")
        self.connect_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }")
        self.connect_btn.clicked.connect(self.toggle_connection)
        connection_layout.addWidget(self.connect_btn)
        
        self.connection_status = QLabel("Bağlantı Yok")
        self.connection_status.setAlignment(Qt.AlignCenter)
        connection_layout.addWidget(self.connection_status)
        
        layout.addWidget(connection_group)
        
        # Veri Gönderme
        send_group = QGroupBox("Veri Gönder")
        send_layout = QVBoxLayout(send_group)
        
        self.send_input = QLineEdit()
        self.send_input.setPlaceholderText("Göndermek istediğiniz veriyi yazın...")
        send_layout.addWidget(self.send_input)
        
        send_btn_layout = QHBoxLayout()
        self.send_btn = QPushButton("Gönder")
        self.send_btn.clicked.connect(self.send_data)
        self.send_btn.setEnabled(False)
        
        self.clear_btn = QPushButton("Temizle")
        self.clear_btn.clicked.connect(self.clear_all_data)
        
        send_btn_layout.addWidget(self.send_btn)
        send_btn_layout.addWidget(self.clear_btn)
        send_layout.addLayout(send_btn_layout)
        
        layout.addWidget(send_group)
        
        # İstatistikler
        stats_group = QGroupBox("İstatistikler")
        stats_layout = QGridLayout(stats_group)
        
        stats_layout.addWidget(QLabel("Alınan Veri:"), 0, 0)
        self.data_count_label = QLabel("0")
        stats_layout.addWidget(self.data_count_label, 0, 1)
        
        stats_layout.addWidget(QLabel("Çalışma Süresi:"), 1, 0)
        self.uptime_label = QLabel("00:00:00")
        stats_layout.addWidget(self.uptime_label, 1, 1)
        
        layout.addWidget(stats_group)
        
        # Sensor Durumu
        sensor_group = QGroupBox("Sensor Durumu")
        sensor_layout = QGridLayout(sensor_group)
        
        # MQ4 Gaz Sensörü
        sensor_layout.addWidget(QLabel("MQ4 Analog:"), 0, 0)
        self.mq4_analog_label = QLabel("0")
        self.mq4_analog_label.setStyleSheet("font-weight: bold;")
        sensor_layout.addWidget(self.mq4_analog_label, 0, 1)
        
        sensor_layout.addWidget(QLabel("MQ4 Voltaj:"), 1, 0)
        self.mq4_voltage_label = QLabel("0.0 V")
        sensor_layout.addWidget(self.mq4_voltage_label, 1, 1)
        
        sensor_layout.addWidget(QLabel("Gaz Tespit:"), 2, 0)
        self.gas_detected_label = QLabel("YOK")
        self.gas_detected_label.setStyleSheet("font-weight: bold; color: green;")
        sensor_layout.addWidget(self.gas_detected_label, 2, 1)
        
        # Çevre Sensörleri
        sensor_layout.addWidget(QLabel("Sıcaklık:"), 3, 0)
        self.temperature_label = QLabel("0.0°C")
        sensor_layout.addWidget(self.temperature_label, 3, 1)
        
        sensor_layout.addWidget(QLabel("Basınç:"), 4, 0)
        self.pressure_label = QLabel("0.0 hPa")
        sensor_layout.addWidget(self.pressure_label, 4, 1)
        
        sensor_layout.addWidget(QLabel("İrtifa:"), 5, 0)
        self.altitude_label = QLabel("0.0 m")
        sensor_layout.addWidget(self.altitude_label, 5, 1)
        
        # GPS Bilgileri
        sensor_layout.addWidget(QLabel("GPS Konum:"), 6, 0)
        self.gps_label = QLabel("0.0, 0.0")
        sensor_layout.addWidget(self.gps_label, 6, 1)
        
        layout.addWidget(sensor_group)
        
        # Ayarlar
        settings_group = QGroupBox("Görüntüleme Ayarları")
        settings_layout = QVBoxLayout(settings_group)
        
        self.auto_scroll_cb = QCheckBox("Otomatik Kaydır")
        self.auto_scroll_cb.setChecked(True)
        settings_layout.addWidget(self.auto_scroll_cb)
        
        self.show_timestamp_cb = QCheckBox("Zaman Damgası Göster")
        self.show_timestamp_cb.setChecked(True)
        settings_layout.addWidget(self.show_timestamp_cb)
        
        layout.addWidget(settings_group)
        
        layout.addStretch()
        return panel
        
    def create_data_panel(self):
        """Veri görüntüleme panelini oluştur"""
        panel = QTabWidget()
        
        # Ham Veri Tab
        raw_data_widget = QWidget()
        raw_layout = QVBoxLayout(raw_data_widget)
        
        self.data_display = QTextEdit()
        self.data_display.setReadOnly(True)
        self.data_display.setFont(QFont("Consolas", 10))
        raw_layout.addWidget(self.data_display)
        
        panel.addTab(raw_data_widget, "Ham Veri")
        
        # Grafik Tab
        self.visualization_widget = MultiChannelDataVisualization()
        panel.addTab(self.visualization_widget, "Grafikler")
        
        # Harita Tab
        self.map_widget = MapVisualization()
        panel.addTab(self.map_widget, "Harita")
        
        # Tablo Tab
        table_widget = QWidget()
        table_layout = QVBoxLayout(table_widget)
        
        self.data_table = QTableWidget()
        self.data_table.setColumnCount(3)
        self.data_table.setHorizontalHeaderLabels(["Zaman", "Sensor Verisi", "Değer"])
        table_layout.addWidget(self.data_table)
        
        panel.addTab(table_widget, "Tablo")
        
        return panel
    
    def apply_modern_style(self):
        """Modern görünüm stilini uygula"""
        style = """
        QMainWindow {
            background-color: #f5f5f5;
        }
        QGroupBox {
            font-weight: bold;
            border: 2px solid #cccccc;
            border-radius: 5px;
            margin-top: 1ex;
            padding-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px 0 5px;
        }
        QPushButton {
            background-color: #e1e1e1;
            border: 1px solid #cccccc;
            border-radius: 4px;
            padding: 5px;
            min-height: 20px;
        }
        QPushButton:hover {
            background-color: #d1d1d1;
        }
        QPushButton:pressed {
            background-color: #c1c1c1;
        }
        QComboBox {
            border: 1px solid #cccccc;
            border-radius: 4px;
            padding: 3px;
            min-height: 20px;
        }
        QLineEdit {
            border: 1px solid #cccccc;
            border-radius: 4px;
            padding: 5px;
            min-height: 20px;
        }
        QTextEdit {
            border: 1px solid #cccccc;
            border-radius: 4px;
            background-color: white;
        }
        """
        self.setStyleSheet(style)
    
    def setup_connections(self):
        """Sinyal-slot bağlantılarını kur"""
        self.serial_thread.data_received.connect(self.handle_received_data)
        self.serial_thread.error_occurred.connect(self.handle_error)
        self.send_input.returnPressed.connect(self.send_data)
        
    def refresh_ports(self):
        """Kullanılabilir seri portları yenile"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")
    
    def toggle_connection(self):
        """Seri port bağlantısını aç/kapat"""
        if not self.serial_thread.running:
            # Bağlan
            if self.port_combo.currentText():
                port = self.port_combo.currentText().split(' - ')[0]
                baudrate = int(self.baudrate_combo.currentText())
                
                if self.serial_thread.setup_serial(port, baudrate):
                    self.serial_thread.start()
                    self.connect_btn.setText("Bağlantıyı Kes")
                    self.connect_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; }")
                    self.connection_status.setText(f"Bağlı: {port}")
                    self.connection_status.setStyleSheet("color: green;")
                    self.send_btn.setEnabled(True)
                    self.status_bar.showMessage(f"Bağlantı kuruldu: {port} @ {baudrate} baud")
            else:
                QMessageBox.warning(self, "Uyarı", "Lütfen bir port seçin!")
        else:
            # Bağlantıyı kes
            self.serial_thread.stop()
            self.serial_thread.wait()
            self.connect_btn.setText("Bağlan")
            self.connect_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }")
            self.connection_status.setText("Bağlantı Yok")
            self.connection_status.setStyleSheet("color: red;")
            self.send_btn.setEnabled(False)
            self.status_bar.showMessage("Bağlantı kesildi")
    
    def handle_received_data(self, data):
        """Alınan veriyi işle"""
        self.received_data_count += 1
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # Ham veri görüntüleme
        if self.show_timestamp_cb.isChecked():
            formatted_data = f"[{timestamp}] {data}"
        else:
            formatted_data = data
            
        self.data_display.append(formatted_data)
        
        if self.auto_scroll_cb.isChecked():
            cursor = self.data_display.textCursor()
            cursor.movePosition(cursor.End)
            self.data_display.setTextCursor(cursor)
        
        # Sensor verilerini parse et
        parsed_data = self.parser.parse_line(data)
        self.current_sensor_data.update(parsed_data)
        
        # JSON veya zaman damgası kontrolü - hem eski hem yeni format için
        is_complete_data = False
        if data.startswith('{') and data.endswith('}'):
            # JSON formatında tam veri
            is_complete_data = True
        elif 'ZAMAN:' in data and any(self.current_sensor_data.values()):
            # Eski formatta zaman damgası
            is_complete_data = True
            
        if is_complete_data:
            self.visualization_widget.add_sensor_data(self.current_sensor_data)
            self.update_sensor_display()
            
            # Harita güncelle
            if (self.current_sensor_data.get('gps_latitude', 0) != 0 or 
                self.current_sensor_data.get('gps_longitude', 0) != 0):
                self.map_widget.update_location(
                    self.current_sensor_data.get('gps_latitude', 0),
                    self.current_sensor_data.get('gps_longitude', 0),
                    self.current_sensor_data.get('gps_altitude', 0),
                    self.current_sensor_data.get('gps_speed', 0),
                    self.current_sensor_data.get('gps_satellites', 0)
                )
        
        # Tablo görüntüleme
        should_add_to_table = False
        extracted_value = "N/A"
        
        # JSON formatını kontrol et
        if data.startswith('{') and data.endswith('}'):
            should_add_to_table = True
            try:
                json_data = json.loads(data)
                key_values = []
                if json_data.get('mq4_analog', 0) > 0:
                    key_values.append(f"MQ4: {json_data['mq4_analog']}")
                if json_data.get('temperature', 0) > 0:
                    key_values.append(f"Sıcaklık: {json_data['temperature']:.1f}°C")
                if json_data.get('gas_detected', False):
                    key_values.append("GAZ TESPİT!")
                extracted_value = " | ".join(key_values) if key_values else "JSON Veri"
            except:
                extracted_value = "JSON Parse Hatası"
        # Eski format
        elif any(key in data.lower() for key in ['zaman:', 'mq4 analog:', 'sıcaklık:', 'gaz durumu:']):
            should_add_to_table = True
            if 'MQ4 Analog:' in data:
                try:
                    extracted_value = data.split(':')[1].strip()
                except:
                    pass
            elif 'Sıcaklık:' in data:
                try:
                    extracted_value = data.split(':')[1].strip()
                except:
                    pass
            elif 'Basınç:' in data:
                try:
                    extracted_value = data.split(':')[1].strip()
                except:
                    pass
        
        if should_add_to_table:
            row_count = self.data_table.rowCount()
            self.data_table.insertRow(row_count)
            self.data_table.setItem(row_count, 0, QTableWidgetItem(timestamp))
            self.data_table.setItem(row_count, 1, QTableWidgetItem(data[:100] + "..." if len(data) > 100 else data))
            self.data_table.setItem(row_count, 2, QTableWidgetItem(extracted_value))
            
            # Tablo boyutunu sınırla
            if self.data_table.rowCount() > 1000:
                self.data_table.removeRow(0)
    
    def handle_error(self, error_msg):
        """Hata mesajlarını işle"""
        self.status_bar.showMessage(f"Hata: {error_msg}")
        QMessageBox.critical(self, "Hata", error_msg)
    
    def send_data(self):
        """Veri gönder"""
        if self.serial_thread.serial_port and self.serial_thread.serial_port.is_open:
            data = self.send_input.text()
            if data:
                try:
                    self.serial_thread.serial_port.write((data + '\n').encode('utf-8'))
                    self.send_input.clear()
                    self.status_bar.showMessage(f"Gönderildi: {data}")
                except Exception as e:
                    QMessageBox.critical(self, "Gönderme Hatası", str(e))
    
    def update_sensor_display(self):
        """Sensor verilerini kontrol panelinde güncelle"""
        # MQ4 Gaz Sensörü verileri
        self.mq4_analog_label.setText(str(self.current_sensor_data.get('mq4_analog', 0)))
        self.mq4_voltage_label.setText(f"{self.current_sensor_data.get('mq4_voltage', 0.0):.2f} V")
        
        # Gaz tespit durumu renkli gösterimi
        gas_detected = self.current_sensor_data.get('gas_detected', False)
        if gas_detected:
            self.gas_detected_label.setText("TESPİT EDİLDİ")
            self.gas_detected_label.setStyleSheet("font-weight: bold; color: red;")
        else:
            self.gas_detected_label.setText("YOK")
            self.gas_detected_label.setStyleSheet("font-weight: bold; color: green;")
        
        # Çevre sensör verileri
        self.temperature_label.setText(f"{self.current_sensor_data.get('temperature', 0.0):.1f}°C")
        self.pressure_label.setText(f"{self.current_sensor_data.get('pressure', 0.0):.1f} hPa")
        self.altitude_label.setText(f"{self.current_sensor_data.get('altitude', 0.0):.1f} m")
        
        # GPS verileri
        gps_lat = self.current_sensor_data.get('gps_latitude', 0.0)
        gps_lon = self.current_sensor_data.get('gps_longitude', 0.0)
        self.gps_label.setText(f"{gps_lat:.6f}, {gps_lon:.6f}")
        
        # Status bar güncellemesi
        timestamp = self.current_sensor_data.get('timestamp', 0)
        if timestamp > 0:
            status_msg = f"Zaman: {timestamp} | "
            status_msg += f"MQ4: {self.current_sensor_data.get('mq4_analog', 0)} | "
            status_msg += f"Sıcaklık: {self.current_sensor_data.get('temperature', 0):.1f}°C | "
            status_msg += f"Gaz: {'TESPİT' if gas_detected else 'YOK'}"
            self.status_bar.showMessage(status_msg)
    
    def clear_all_data(self):
        """Tüm verileri temizle"""
        self.data_display.clear()
        self.data_table.setRowCount(0)
        self.visualization_widget.clear_data()
        self.map_widget.clear_gps_history()
        self.received_data_count = 0
        self.start_time = time.time()
        self.current_sensor_data.clear()
        self.parser.reset_data()
        self.status_bar.showMessage("Veriler temizlendi")
    
    def update_status(self):
        """Durum bilgilerini güncelle"""
        self.data_count_label.setText(str(self.received_data_count))
        
        uptime_seconds = int(time.time() - self.start_time)
        hours = uptime_seconds // 3600
        minutes = (uptime_seconds % 3600) // 60
        seconds = uptime_seconds % 60
        self.uptime_label.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")
    
    def closeEvent(self, event):
        """Uygulama kapanırken temizlik yap"""
        if self.serial_thread.running:
            self.serial_thread.stop()
            self.serial_thread.wait()
        event.accept()


def main() -> None:
    """Ana fonksiyon"""
    app = QApplication(sys.argv)
    
    # Uygulama ikonu ve başlığı
    app.setApplicationName("Seri Port İzleyici")
    app.setApplicationVersion("1.0")
    
    # Ana pencereyi oluştur ve göster
    window = SerialMonitorApp()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
