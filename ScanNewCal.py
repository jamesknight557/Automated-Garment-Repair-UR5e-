# Camera 20cm from the felt
# Detect holes in the felt and generate tool paths for robotic felting repair

import sys
import cv2
import numpy as np
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                            QLabel, QVBoxLayout, QWidget, QHBoxLayout, 
                            QSpacerItem, QSizePolicy, QTextEdit, QSlider,
                            QRadioButton, QButtonGroup)
from PyQt5.QtGui import QImage, QPixmap, QFont, QPalette, QColor
from PyQt5.QtCore import Qt, QSize
import threading
import ezdxf  # Add this import at the top of your file

import URBasic
import URBasic.robotModel
import URBasic.urScriptExt

class FeltingRepairApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Felting Repair System")
        self.setGeometry(100, 20, 1920, 1080)  # Move window up and set size (optional)
        self.showFullScreen()
        
        # Initialize variables
        self.original_image = None
        self.detected_holes = []
        self.tool_paths = []
        self.pixel_to_mm = None
        self.workspace_diameter_mm = 142  # Known diameter of workspace in mm
        self.workspace_center = None
        self.workspace_radius = None
        self.calibration_image = None
        
        # Set dark theme
        self.set_dark_theme()
        
        # Main widget and layout
        self.central_widget = QWidget()
        self.main_layout = QVBoxLayout()
        self.main_layout.setContentsMargins(20, 20, 20, 20)
        self.main_layout.setSpacing(8)  # Reduce spacing between widgets
        
        # Header with title
        self.create_header()
        
        # Image display area
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("""
            background-color: #2d2d2d;
            border: 2px solid #444;
            border-radius: 10px;
        """)
        self.image_label.setFixedSize(1875, 850)  # Reduce image area height to make more space for buttons
        self.main_layout.addWidget(self.image_label, 1)
        
        # Text output area
        self.text_output = QTextEdit()
        self.text_output.setReadOnly(True)
        self.text_output.setStyleSheet("""
            background-color: #252525;
            color: #ffffff;
            border: 1px solid #444;
            border-radius: 5px;
            padding: 10px;
        """)
        self.text_output.setMaximumHeight(120)  # Reduce text output height to make more space
        self.main_layout.addWidget(self.text_output)
        
        # --- Add sliders for kernel size and iterations ---
        slider_panel = QWidget()
        slider_layout = QHBoxLayout()

        # Kernel Size Slider
        self.kernel_label = QLabel("Kernel Size: 4")
        self.kernel_slider = QSlider(Qt.Horizontal)
        self.kernel_slider.setMinimum(1)
        self.kernel_slider.setMaximum(5)
        self.kernel_slider.setValue(4)
        self.kernel_slider.setTickPosition(QSlider.TicksBelow)
        self.kernel_slider.setTickInterval(1)
        self.kernel_slider.valueChanged.connect(lambda v: self.kernel_label.setText(f"Kernel Size: {v}"))
        slider_layout.addWidget(self.kernel_label)
        slider_layout.addWidget(self.kernel_slider)

        # Iterations Slider
        self.iter_label = QLabel("Iterations: 2")
        self.iter_slider = QSlider(Qt.Horizontal)
        self.iter_slider.setMinimum(1)
        self.iter_slider.setMaximum(5)
        self.iter_slider.setValue(2)
        self.iter_slider.setTickPosition(QSlider.TicksBelow)
        self.iter_slider.setTickInterval(1)
        self.iter_slider.valueChanged.connect(lambda v: self.iter_label.setText(f"Iterations: {v}"))
        slider_layout.addWidget(self.iter_label)
        slider_layout.addWidget(self.iter_slider)

        slider_panel.setLayout(slider_layout)
        self.main_layout.addWidget(slider_panel)
        self.main_layout.addSpacing(5)  # Add a small space before the buttons
        
        # --- Add toolpath shape selection ---
        shape_panel = QWidget()
        shape_layout = QHBoxLayout()
        shape_label = QLabel("Repair Shape:")
        shape_label.setStyleSheet("color: #fff; font-weight: bold;")
        shape_layout.addWidget(shape_label)

        self.shape_group = QButtonGroup(self)
        self.radio_circle = QRadioButton("Circular")
        self.radio_square = QRadioButton("Square")
        self.radio_cross = QRadioButton("Cross")
        self.radio_circle.setChecked(True)  # Default

        self.shape_group.addButton(self.radio_circle)
        self.shape_group.addButton(self.radio_square)
        self.shape_group.addButton(self.radio_cross)

        shape_layout.addWidget(self.radio_circle)
        shape_layout.addWidget(self.radio_square)
        shape_layout.addWidget(self.radio_cross)
        shape_panel.setLayout(shape_layout)
        self.main_layout.addWidget(shape_panel)
        
        # Button panel
        self.create_button_panel()
        
        # Calibration buttons (initially hidden)
        self.capture_calib_btn = QPushButton(" CAPTURE CALIBRATION IMAGE ")
        self.capture_calib_btn.setFont(QFont('Arial', 12))
        self.capture_calib_btn.setStyleSheet(self.get_button_style("#4CAF50"))
        self.capture_calib_btn.clicked.connect(self.capture_calibration_image)
        self.capture_calib_btn.hide()
        self.main_layout.addWidget(self.capture_calib_btn)
        
        self.confirm_btn = QPushButton(" CONFIRM CALIBRATION ")
        self.confirm_btn.setFont(QFont('Arial', 12))
        self.confirm_btn.setStyleSheet(self.get_button_style("#4CAF50"))
        self.confirm_btn.clicked.connect(self.confirm_calibration)
        self.confirm_btn.hide()
        self.main_layout.addWidget(self.confirm_btn)
        
        self.central_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.central_widget)
        
        # Show initial placeholder
        self.show_placeholder()
    
    def set_dark_theme(self):
        """Set a dark theme for the application"""
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.WindowText, Qt.white)
        palette.setColor(QPalette.Base, QColor(25, 25, 25))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, Qt.white)
        palette.setColor(QPalette.ToolTipText, Qt.white)
        palette.setColor(QPalette.Text, Qt.white)
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, Qt.white)
        palette.setColor(QPalette.BrightText, Qt.red)
        palette.setColor(QPalette.Highlight, QColor(142, 45, 197).lighter())
        palette.setColor(QPalette.HighlightedText, Qt.black)
        self.setPalette(palette)

    def create_header(self):
        """Create the header with title"""
        header = QWidget()
        header_layout = QHBoxLayout()
        
        title = QLabel("ROBOTIC FELTING REPAIR SYSTEM")
        title.setFont(QFont('Arial', 24, QFont.Bold))
        title.setStyleSheet("color: #8e2dc5;")
        title.setAlignment(Qt.AlignCenter)
        
        header_layout.addWidget(title)
        self.main_layout.addWidget(header)
    
    def create_button_panel(self):
        """Create the button panel"""
        button_panel = QWidget()
        button_layout = QHBoxLayout()
        
        # Spacers to center buttons
        button_layout.addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        
        self.calibrate_btn = QPushButton(" CALIBRATE CAMERA ")
        self.calibrate_btn.setFont(QFont('Arial', 12))
        self.calibrate_btn.setStyleSheet(self.get_button_style("#2d7dc5"))
        self.calibrate_btn.clicked.connect(self.start_calibration)
        button_layout.addWidget(self.calibrate_btn)
        
        self.capture_btn = QPushButton(" DETECT HOLES ")
        self.capture_btn.setFont(QFont('Arial', 12))
        self.capture_btn.setStyleSheet(self.get_button_style("#8e2dc5"))
        self.capture_btn.clicked.connect(self.capture_and_process)
        self.capture_btn.setEnabled(False)
        button_layout.addWidget(self.capture_btn)
        
        self.toolpath_btn = QPushButton(" GENERATE TOOL PATH ")
        self.toolpath_btn.setFont(QFont('Arial', 12))
        self.toolpath_btn.setStyleSheet(self.get_button_style("#2dc58e"))
        self.toolpath_btn.clicked.connect(self.generate_tool_path)
        self.toolpath_btn.setEnabled(False)
        button_layout.addWidget(self.toolpath_btn)

        # Add the new "Repair Hole" button
        self.repair_btn = QPushButton(" REPAIR HOLE ")
        self.repair_btn.setFont(QFont('Arial', 12))
        self.repair_btn.setStyleSheet(self.get_button_style("#e67e22"))
        self.repair_btn.clicked.connect(self.repair_hole)
        self.repair_btn.setEnabled(False)
        button_layout.addWidget(self.repair_btn)

        # Add the confirm calibration button here, but hide it by default
        self.confirm_btn = QPushButton(" CONFIRM CALIBRATION ")
        self.confirm_btn.setFont(QFont('Arial', 12))
        self.confirm_btn.setStyleSheet(self.get_button_style("#4CAF50"))
        self.confirm_btn.clicked.connect(self.confirm_calibration)
        self.confirm_btn.hide()
        button_layout.addWidget(self.confirm_btn)
        
        # Spacers to center buttons
        button_layout.addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        
        button_panel.setLayout(button_layout)
        self.main_layout.addWidget(button_panel)
        self.main_layout.addStretch(1)  # Push everything up by adding stretch at the end
    
    def get_button_style(self, color):
        """Generate button style sheet"""
        return f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border: none;
                border-radius: 15px;
                padding: 12px 25px;
                min-width: 180px;
            }}
            QPushButton:hover {{
                background-color: {self.lighten_color(color)};
            }}
            QPushButton:pressed {{
                background-color: {self.darken_color(color)};
            }}
            QPushButton:disabled {{
                background-color: #555555;
                color: #aaaaaa;
            }}
        """
    
    def lighten_color(self, hex_color, factor=0.2):
        """Lighten a color"""
        color = QColor(hex_color)
        return color.lighter(int(100 + (100 * factor))).name()
    
    def darken_color(self, hex_color, factor=0.2):
        """Darken a color"""
        color = QColor(hex_color)
        return color.darker(int(100 + (100 * factor))).name()
    
    def show_placeholder(self):
        """Show initial placeholder image"""
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        blank[:] = (45, 45, 45)
        cv2.putText(blank, "First click 'Calibrate Camera' to set up", (100, 200), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(blank, "Then click 'Detect Holes' to find damage", (100, 240), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
        cv2.putText(blank, "Finally click 'Generate Tool Path' for repair", (90, 280), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
        # Show placeholder without crosshair
        self.display_image(blank, draw_crosshair=False)

    def display_image(self, cv_img, draw_crosshair=True):
        """Display OpenCV image in the QLabel, with optional center green crosshair"""
        h, w = cv_img.shape[:2]
        if draw_crosshair:
            center = (w // 2, h // 2)
            crosshair_length = 30
            crosshair_color = (0, 255, 0)
            crosshair_thickness = 2
            # Draw horizontal line
            cv2.line(cv_img, (center[0] - crosshair_length, center[1]), (center[0] + crosshair_length, center[1]), crosshair_color, crosshair_thickness)
            # Draw vertical line
            cv2.line(cv_img, (center[0], center[1] - crosshair_length), (center[0], center[1] + crosshair_length), crosshair_color, crosshair_thickness)

        # Convert color (BGR to RGB)
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        # Scale to fit label while maintaining aspect ratio
        pixmap = QPixmap.fromImage(qt_image)
        self.image_label.setPixmap(pixmap.scaled(
            self.image_label.width(), self.image_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation
        ))

    def start_calibration(self):
        """Start the calibration process with a live camera feed"""
        self.calibrate_btn.setEnabled(False)
        self.capture_btn.setEnabled(False)
        self.toolpath_btn.setEnabled(False)
        self.capture_calib_btn.hide()
        self.confirm_btn.show()
        self.text_output.clear()
        self.text_output.append("Calibration started - align the real workspace with the yellow circle and press the green banner at the bottom of the screen.")

        # Start live feed in a separate thread to avoid blocking the UI
        self._calib_running = True
        self._calib_frame = None
        self._calib_radius = None

        def calib_loop():
            cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            while self._calib_running:
                ret, frame = cap.read()
                if not ret:
                    continue
                h, w = frame.shape[:2]
                center = (w // 2, h // 2)
                # Make the circle 1.5x bigger than before
                base_radius = min(w, h) // 4
                radius = int(base_radius * 2) if self._calib_radius is None else self._calib_radius

                # Draw yellow circle overlay (no text)
                display = frame.copy()
                cv2.circle(display, center, radius, (0, 255, 255), 3)

                # Save current frame for confirmation
                self._calib_frame = frame.copy()
                self._calib_radius = radius

                # Show in PyQt label
                self.display_image(display)
                cv2.waitKey(30)
            cap.release()

        self._calib_thread = threading.Thread(target=calib_loop, daemon=True)
        self._calib_thread.start()

    def confirm_calibration(self):
        """Confirm the current calibration using the last live frame"""
        try:
            self._calib_running = False
            if not hasattr(self, "_calib_frame") or self._calib_frame is None:
                raise Exception("No calibration frame available")
            frame = self._calib_frame
            h, w = frame.shape[:2]
            center = (w // 2, h // 2)
            radius = self._calib_radius if self._calib_radius else min(w, h) // 4

            self.workspace_center = center
            self.workspace_radius = radius
            self.pixel_to_mm = self.workspace_diameter_mm / (2 * radius)

            self.confirm_btn.hide()
            self.calibrate_btn.setEnabled(True)
            self.capture_btn.setEnabled(True)

            self.text_output.append(f"Calibration complete: {self.pixel_to_mm:.4f} mm/pixel")
            self.text_output.append(f"Workspace center: {center}, radius: {radius}px")

            # Skip showing the "Calibration Successful!" image and placeholder

        except Exception as e:
            self.text_output.append(f"Error confirming calibration: {str(e)}")
            self.confirm_btn.setEnabled(True)
    
    def capture_calibration_image(self):
        """Capture an image for calibration"""
        try:
            # Initialize camera
            cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            
            # Warm-up frames
            for _ in range(5):
                cap.read()
            
            # Capture frame
            ret, frame = cap.read()
            cap.release()
            
            if not ret:
                raise Exception("Failed to capture calibration image")
            
            self.calibration_image = frame.copy()
            
            # Add circle overlay to the captured image
            h, w = self.calibration_image.shape[:2]
            center = (w//2, h//2)
            radius = min(w, h) // 4  # Initial estimate
            
            # Draw circle overlay
            overlay = self.calibration_image.copy()
            cv2.circle(overlay, center, radius, (0, 255, 255), 2)
            
            # Add text overlay
            cv2.putText(overlay, "Align your 150mm circle with yellow circle", 
                       (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(overlay, "Adjust camera position and press Confirm", 
                       (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Blend overlay
            cv2.addWeighted(overlay, 0.7, self.calibration_image, 0.3, 0, self.calibration_image)
            
            # Display the captured image with overlay
            self.display_image(self.calibration_image)
            
            # Show confirm button
            self.capture_calib_btn.hide()
            self.confirm_btn.show()
            self.text_output.append("Calibration image captured - align your circle with the yellow one and press Confirm")
            
        except Exception as e:
            self.text_output.append(f"Error during calibration capture: {str(e)}")
            self.capture_calib_btn.setEnabled(True)
    
    def capture_and_process(self):
        """Capture and process image from webcam"""
        self.capture_btn.setEnabled(False)
        self.capture_btn.setText(" PROCESSING... ")
        self.toolpath_btn.setEnabled(False)
        self.toolpath_btn.setText(" GENERATE TOOL PATH ")  # Reset toolpath button text
        self.repair_btn.setEnabled(False)  # Disable repair button when detecting holes
        QApplication.processEvents()
        
        try:
            # Initialize camera
            cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            
            # Warm-up frames
            for _ in range(5):
                cap.read()
            
            # Capture frame
            ret, frame = cap.read()
            cap.release()
            
            if not ret:
                raise Exception("Failed to capture image")
            
            # Store original image
            self.original_image = frame.copy()
            self.detected_holes = []
            processed_image = frame.copy()
            
            # Draw workspace boundary if calibrated
            if self.workspace_center and self.workspace_radius:
                cv2.circle(processed_image, 
                          (self.workspace_center[0], self.workspace_center[1]), 
                          self.workspace_radius, (0, 255, 255), 4)
            
            # Convert to grayscale and threshold
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 155, 255, cv2.THRESH_BINARY)

            #cv2.imshow("Thresholded Image", thresh)  # Debugging line
            
            # Noise removal
            kernel_size = self.kernel_slider.value()
            iterations = self.iter_slider.value()
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=iterations)

            #cv2.imshow("Cleaned Image", cleaned)  # Debugging line
            
            # Find contours
            contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.last_contours = contours  # <-- Add this line
            
            # Store hole information and draw on image
            for i, cnt in enumerate(contours):
                area = cv2.contourArea(cnt)
                if area > 200 and area < 10000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    center = (x + w//2, y + h//2)
                    
                    # Only process holes within workspace if calibrated
                    if not self.workspace_center or self.is_point_in_workspace(center):
                        self.detected_holes.append({
                            'id': i+1,
                            'x': x,
                            'y': y,
                            'width': w,
                            'height': h,
                            'center_x': center[0],
                            'center_y': center[1],
                            'diameter': max(w, h)
                        })
                        cv2.rectangle(processed_image, (x,y), (x+w,y+h), (0,255,0), 4)
                        cv2.putText(processed_image, f"Hole {i+1}", (x+5,y-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                elif area >= 2000:
                    # Only process large holes within workspace if calibrated
                    x, y, w, h = cv2.boundingRect(cnt)
                    center = (x + w//2, y + h//2)
                    if not self.workspace_center or self.is_point_in_workspace(center):
                        # Save DXF of the original outline
                        dxf_doc = ezdxf.new(dxfversion="R2010")
                        msp = dxf_doc.modelspace()
                        # Convert contour to a list of tuples
                        points = [tuple(pt[0]) for pt in cnt]
                        # Close the polyline if not already closed
                        if points[0] != points[-1]:
                            points.append(points[0])
                        msp.add_lwpolyline(points, close=True)
                        dxf_filename = f"large_hole_{i+1}.dxf"
                        dxf_doc.saveas(dxf_filename)
                        self.text_output.append(f"Hole {i+1} is too large for felting. Outline saved as {dxf_filename}.")

                        # --- Create and save a 50% bigger contour as a patch outline ---
                        # Calculate centroid of the contour
                        M = cv2.moments(cnt)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                        else:
                            cx, cy = center  # fallback

                        # Scale contour points 50% further from centroid
                        bigger_cnt = []
                        for pt in cnt:
                            px, py = pt[0]
                            new_x = int(cx + 1.8 * (px - cx))
                            new_y = int(cy + 1.8 * (py - cy))
                            bigger_cnt.append((new_x, new_y))
                        # Close the polyline if not already closed
                        if bigger_cnt[0] != bigger_cnt[-1]:
                            bigger_cnt.append(bigger_cnt[0])

                        # Save bigger contour as DXF
                        patch_dxf_doc = ezdxf.new(dxfversion="R2010")
                        patch_msp = patch_dxf_doc.modelspace()
                        patch_msp.add_lwpolyline(bigger_cnt, close=True)
                        patch_dxf_filename = f"large_hole_{i+1}_patch.dxf"
                        patch_dxf_doc.saveas(patch_dxf_filename)
                        self.text_output.append(f"Patch outline (50% bigger) saved as {patch_dxf_filename}.")

                        # Draw the large hole in red
                        cv2.drawContours(processed_image, [cnt], -1, (0, 0, 255), 4)
                        cv2.putText(processed_image, f"Large Hole {i+1}", (x+5, y-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        # Optionally, show a message box to the user
                        """ from PyQt5.QtWidgets import QMessageBox
                        QMessageBox.warning(self, "Large Hole Detected",
                            f"Hole {i+1} is too large for felting.\nOutline saved as {dxf_filename}.\n"
                            f"Patch outline saved as {patch_dxf_filename}.\nPlease cut out on laser cutter.") """
            
            self.display_image(processed_image)
            self.text_output.clear()
            self.text_output.append(f"Detected {len(self.detected_holes)} holes")
            
            if self.detected_holes:
                self.toolpath_btn.setEnabled(True)
            
        except Exception as e:
            self.text_output.append(f"Error: {str(e)}")
            error_img = np.zeros((480, 640, 3), dtype=np.uint8)
            error_img[:] = (45, 45, 45)
            cv2.putText(error_img, f"Error: {str(e)}", (50, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.display_image(error_img)
        
        finally:
            self.capture_btn.setEnabled(True)
            self.capture_btn.setText(" DETECT HOLES ")
    
    def is_point_in_workspace(self, point):
        """Check if a point is within the workspace circle"""
        if not self.workspace_center or not self.workspace_radius:
            return True  # If not calibrated, assume all points are in workspace
        distance = np.sqrt((point[0] - self.workspace_center[0])**2 + 
                   (point[1] - self.workspace_center[1])**2)
        return distance <= self.workspace_radius
    
    def generate_tool_path(self):
        """Generate filled circular needle paths around each hole with connected points"""
        if not self.detected_holes or self.original_image is None:
            self.text_output.append("No image or holes detected")
            self.repair_btn.setEnabled(False)  # Disable repair button if no holes
            return
            
        self.toolpath_btn.setEnabled(False)
        self.toolpath_btn.setText(" GENERATED ")
        self.repair_btn.setEnabled(False)  # Disable repair button while generating
        QApplication.processEvents()
        
        try:
            # Start with original image
            toolpath_image = self.original_image.copy()
            self.tool_paths = []
            needle_count = 0

            # Get image center for offset
            h, w = toolpath_image.shape[:2]
            image_center = (w // 2, h // 2)
            
            # Draw workspace boundary if calibrated
            if self.workspace_center and self.workspace_radius:
                cv2.circle(toolpath_image, 
                          (self.workspace_center[0], self.workspace_center[1]), 
                          self.workspace_radius, (0, 255, 255), 2)
            
            # Determine toolpath mode from selected radio button
            if self.radio_circle.isChecked():
                toolpath_mode = "circle"
            elif self.radio_square.isChecked():
                toolpath_mode = "square"
            elif self.radio_cross.isChecked():
                toolpath_mode = "cross"
            else:
                toolpath_mode = "circle"

            # Generate tool paths for each detected hole
            for hole in self.detected_holes:
                center = (hole['center_x'], hole['center_y'])
                all_needles = []

                if toolpath_mode == "circle":
                    # Calculate concentric circles parameters
                    max_radius = max(hole['width'], hole['height'])
                    min_radius = max(10, min(hole['width'], hole['height']) // 2)
                    
                    # Generate multiple concentric circles
                    circles = []
                    for r in np.linspace(min_radius, max_radius, num=3):
                        num_needles = max(6, min(36, int(2 * np.pi * r / 20)))  # Generate 6 to 36 needles based on radius
                        needles = []
                        for i in range(num_needles):
                            angle = 2 * np.pi * i / num_needles
                            x = int(center[0] + r * np.cos(angle))
                            y = int(center[1] + r * np.sin(angle))
                            
                            # Only include points within workspace if calibrated
                            if not self.workspace_center or self.is_point_in_workspace((x, y)):
                                needles.append((x, y))
                        circles.append(needles)
                    # Connect needles in spiral pattern
                    for i, circle in enumerate(circles):
                        if i % 2 == 1:
                            circle = circle[len(circle)//2:] + circle[:len(circle)//2]
                        all_needles.extend(circle)

                elif toolpath_mode == "square":
                    size = 3 * max(hole['width'], hole['height'])  # Make square twice as big
                    step = max(6, min(6, size // 100))
                    for layer in range(2):
                        offset = int((size // 2) * (layer / 2))
                        points = []
                        for t in np.linspace(0, 1, step, endpoint=False):
                            # Top edge
                            x = center[0] - size//2 + offset + int(t * (size - 2*offset))
                            y = center[1] - size//2 + offset
                            points.append((x, y))
                        for t in np.linspace(0, 1, step, endpoint=False):
                            # Right edge
                            x = center[0] + size//2 - offset
                            y = center[1] - size//2 + offset + int(t * (size - 2*offset))
                            points.append((x, y))
                        for t in np.linspace(0, 1, step, endpoint=False):
                            # Bottom edge
                            x = center[0] + size//2 - offset - int(t * (size - 2*offset))
                            y = center[1] + size//2 - offset
                            points.append((x, y))
                        for t in np.linspace(0, 1, step, endpoint=False):
                            # Left edge
                            x = center[0] - size//2 + offset
                            y = center[1] + size//2 - offset - int(t * (size - 2*offset))
                            points.append((x, y))
                        all_needles.extend(points)

                elif toolpath_mode == "cross":
                    size = 3 * max(hole['width'], hole['height'])  # Make cross three times as big
                    step = max(6, min(36, size // 8))
                    # Horizontal line
                    for t in np.linspace(-size//2, size//2, step):
                        x = int(center[0] + t)
                        y = center[1]
                        if not self.workspace_center or self.is_point_in_workspace((x, y)):
                            all_needles.append((x, y))
                    # Vertical line
                    for t in np.linspace(-size//2, size//2, step):
                        x = center[0]
                        y = int(center[1] + t)
                        if not self.workspace_center or self.is_point_in_workspace((x, y)):
                            all_needles.append((x, y))

                # Draw needle points first (blue)
                for x, y in all_needles:
                    cv2.circle(toolpath_image, (x, y), 4, (255, 0, 0), -1)
                
                # Then draw connected path over them (orange)
                for i in range(len(all_needles)-1):
                    cv2.line(toolpath_image, all_needles[i], all_needles[i+1], (0, 165, 255), 2)
                
                # Connect last to first to complete the circle
                if len(all_needles) > 1:
                    cv2.line(toolpath_image, all_needles[-1], all_needles[0], (0, 165, 255), 2)
                
                # Store tool path data, offset from image center
                self.tool_paths.append({
                    'hole_id': hole['id'],
                    'center': center,
                    'needles': all_needles,
                    'center_mm': (
                        (hole['center_x'] - image_center[0]) * self.pixel_to_mm if self.pixel_to_mm else 0,
                        (hole['center_y'] - image_center[1]) * self.pixel_to_mm if self.pixel_to_mm else 0
                    )
                })
                needle_count += len(all_needles)
            
            # Redraw hole boxes over the tool paths
            for hole in self.detected_holes:
                cv2.rectangle(toolpath_image, 
                            (hole['x'], hole['y']), 
                            (hole['x']+hole['width'], hole['y']+hole['height']), 
                            (0, 255, 0), 2)
                cv2.putText(toolpath_image, f"Hole {hole['id']}", 
                           (hole['x']+5, hole['y']-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # --- Overlay the bigger patch for large holes ---
            for i, cnt in enumerate(self.last_contours):  # <-- Use self.last_contours
                area = cv2.contourArea(cnt)
                if area >= 2000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    center = (x + w//2, y + h//2)
                    if not self.workspace_center or self.is_point_in_workspace(center):
                        # Calculate centroid of the contour
                        M = cv2.moments(cnt)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                        else:
                            cx, cy = center  # fallback

                        # Scale contour points 50% further from centroid
                        bigger_cnt = []
                        for pt in cnt:
                            px, py = pt[0]
                            new_x = int(cx + 1.8 * (px - cx))
                            new_y = int(cy + 1.8 * (py - cy))
                            bigger_cnt.append((new_x, new_y))
                        # Close the polyline if not already closed
                        if bigger_cnt[0] != bigger_cnt[-1]:
                            bigger_cnt.append(bigger_cnt[0])

                        # Draw the bigger patch as a magenta outline
                        pts_np = np.array([bigger_cnt], dtype=np.int32)
                        cv2.polylines(toolpath_image, pts_np, isClosed=True, color=(255, 0, 255), thickness=3)

            # Display the image with tool paths (with crosshair)
            self.display_image(toolpath_image)
            
            # Output tool path information
            self.text_output.append("\nGenerated Tool Paths:")
            self.text_output.append(f"Total holes: {len(self.tool_paths)}")
            self.text_output.append(f"Total needle positions: {needle_count}")
            self.text_output.append("\nNeedle positions (x, y pixels):")
            
            for path in self.tool_paths:
                self.text_output.append(
                    f"\nHole {path['hole_id']} (Center: {path['center']} pixels, "
                    f"{path['center_mm'][0]:.2f}x{path['center_mm'][1]:.2f} mm from image center):"
                )
                for i, (x, y) in enumerate(path['needles']):
                    dx_mm = (x - image_center[0]) * self.pixel_to_mm if self.pixel_to_mm else 0
                    dy_mm = (y - image_center[1]) * self.pixel_to_mm if self.pixel_to_mm else 0
                    self.text_output.append(
                        f"  Needle {i+1}: ({x}, {y}) pixels | ({dx_mm:.2f}, {dy_mm:.2f}) mm from center"
                    )

            # Enable the repair button after successful tool path generation
            self.repair_btn.setEnabled(True)

        except Exception as e:
            self.text_output.append(f"Error generating tool path: {str(e)}")
            import traceback
            traceback.print_exc()
            self.repair_btn.setEnabled(False)  # Disable repair button on error
    
    def repair_hole(self):
        """Move robot to each needle point for all holes using generated tool paths"""

         # --- Switch to original image for live feedback in the Qt widget ---
        live_feedback_img = self.original_image.copy()
        self.display_image(live_feedback_img, draw_crosshair=False)  # Update the QLabel to show the original image

        try:

            # Prompt user to move the robot to Center of the workspace with a popup message
            from PyQt5.QtWidgets import QMessageBox
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)    
            msg.setText("Move the robot to the center of the workspace and press OK.")
            msg.setWindowTitle("Move Robot")
            msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            msg.setDefaultButton(QMessageBox.Ok)
            response = msg.exec_()

            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)    
            msg.setText("Please put robot in remote mode and press OK to continue.")
            msg.setWindowTitle("Remote Mode")
            msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            msg.setDefaultButton(QMessageBox.Ok)
            response = msg.exec_()
            
            time.sleep(1)  # Wait for robot to stabilize

            host = '169.254.130.40'  # arm IP
            
            robotModel = URBasic.robotModel.RobotModel()
            robot = URBasic.urScriptExt.UrScriptExt(host, robotModel)

            robot.reset_error()
            self.text_output.clear()
            self.text_output.append("Connected to robot")

            reset_position = robot.get_actual_tcp_pose()

            time.sleep(1)  # Wait for robot to stabilize


            position = robot.get_actual_tcp_pose()  # Get current TCP pose

            print("Current TCP position:", position)

            # Add 50mm to the z position (move up)
            position[2] += 0.05  # Move up by 50mm

            self.text_output.append("Moving to above felt position...")
            robot.movel(pose=position, a=0.5, v=0.5, wait=True)

            print("Moved to above felt position:", position)

            # Move to all needle positions for all holes
            if not self.tool_paths:
                self.text_output.append("No tool paths generated.")
                robot.close()
                return

            # --- TEST MOVEMENT SEQUENCE ---
            # Move to each hole center, go down 20mm in z, then back up
            
            
            """ for path in self.tool_paths:
                center_x, center_y = path['center']
                # Flip the coordinates to match the real world (mirror horizontally)
                flipped_x = center_x
                flipped_y = center_y

                dx_mm = (flipped_x - self.original_image.shape[1] // 2) * self.pixel_to_mm if self.pixel_to_mm else 0
                dy_mm = (flipped_y - self.original_image.shape[0] // 2) * self.pixel_to_mm if self.pixel_to_mm else 0

                # Move above hole center5
                target_pose = position.copy()
                target_pose[0] += (dy_mm) / 1000.0  # mm to meters
                target_pose[1] += (dx_mm) / 1000.0
                robot.movel(pose=target_pose, a=0.5, v=0.05, wait=True)
                self.text_output.append(f"Moved to center of Hole {path['hole_id']} at (image: {center_x}, {center_y}, robot: {flipped_x}, {flipped_y})")

                # Draw blue dot for feedback
                cv2.circle(live_feedback_img, (center_x, center_y), 6, (255, 0, 0), -1)
                self.display_image(live_feedback_img, draw_crosshair=False)  # Update the QLabel after each move

                # Move down 20mm in z
                down_pose = target_pose.copy()
                down_pose[2] -= 0.080  # Go down 50mm
                robot.movel(pose=down_pose, a=0.5, v=0.05, wait=True)
                self.text_output.append(f"Moved down 50mm at Hole {path['hole_id']}")

                # Move back up
                robot.movel(pose=target_pose, a=0.5, v=0.05, wait=True)
                self.text_output.append(f"Moved back up at Hole {path['hole_id']}")

            self.text_output.append("Test movement sequence complete.") """
            
            
            # ---  MOVEMENT  --- #
            
            for path in self.tool_paths:
                self.text_output.append(f"Repairing Hole {path['hole_id']}...")
                for idx, (x, y) in enumerate(path['needles']):
                    # Convert image (x, y) to robot coordinates (relative to image center)
                    dx_mm = (x - self.original_image.shape[1] // 2) * self.pixel_to_mm if self.pixel_to_mm else 0
                    dy_mm = (y - self.original_image.shape[0] // 2) * self.pixel_to_mm if self.pixel_to_mm else 0

                    # Copy the current pose and update x, y (leave z, rx, ry, rz unchanged)
                    target_pose = position.copy()
                    target_pose[0] += (dy_mm) / 1000.0  # Convert mm to meters
                    target_pose[1] += (dx_mm) / 1000.0

                    robot.movel(pose=target_pose, a=1, v=1, wait=True)
                    
                    # Draw blue dot for feedback
                    cv2.circle(live_feedback_img, (x, y), 6, (255, 0, 0), -1)
                    self.display_image(live_feedback_img, draw_crosshair=False)  # Update the QLabel after each move

                    # Move down 5mm in z
                    down_pose = target_pose.copy()
                    down_pose[2] -= 0.075  # Go down 
                    robot.movel(pose=down_pose, a=1, v=1, wait=True)
                    

                    # Move back up
                    robot.movel(pose=target_pose, a=1, v=1, wait=True)


            self.text_output.append("Repair sequence complete.")
            

            robot.movel(pose=reset_position, a=0.1, v=0.1, wait=True)  # Return to reset position

            robot.close()

        except Exception as e:
            self.text_output.append(f"Error during repair: {str(e)}")
            import traceback
            traceback.print_exc()  # Print stack trace for debugging
    
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = FeltingRepairApp()
    window.show()
    sys.exit(app.exec_())

    #TO DO

    #Different materials sliders


    #Inspection station -> sewing station