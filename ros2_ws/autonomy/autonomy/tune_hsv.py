#!/usr/bin/env python3
"""
tune_hsv.py - Interactive HSV & Threshold Tuning Utility for WRO 2026

Built with Tkinter for smooth, zero-flicker rendering on Linux KDE Plasma / Wayland.

Usage:
    python3 tune_hsv.py --image path/to/sample.jpg
    python3 tune_hsv.py --webcam 0
"""

import argparse
import os
import sys
import tkinter as tk
from tkinter import ttk, messagebox
import cv2
import numpy as np
from PIL import Image, ImageTk

# Ensure local script directory is on sys.path for direct python3 invocation
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

try:
    from autonomy.pillar_detector import PillarDetector
except ModuleNotFoundError:
    from pillar_detector import PillarDetector


class HSVTuningApp:
    """
    Tkinter-based HSV parameter tuning application designed to replace glitchy
    OpenCV HighGUI windows on Linux KDE Plasma / Wayland environments.
    """

    def __init__(self, root: tk.Tk, image_path: str = None, webcam_id: int = None):
        self.root = root
        self.root.title("WRO 2026 - HSV Vision Tuning Tool (KDE/Wayland Ready)")
        self.root.geometry("1300x820")
        self.root.minsize(1000, 650)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.detector = PillarDetector()
        self.cap = None
        self.frame_static = None
        self.is_running = True

        if image_path:
            self.frame_static = cv2.imread(image_path)
            if self.frame_static is None:
                messagebox.showerror("Error", f"Could not load image: {image_path}")
                sys.exit(1)
        elif webcam_id is not None:
            # Force V4L2 backend for Linux and set a fast, low resolution
            self.cap = cv2.VideoCapture(webcam_id, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            if not self.cap.isOpened():
                messagebox.showerror("Error", f"Could not open webcam device {webcam_id}")
                sys.exit(1)

        self._build_ui()
        self.root.after(100, self.update_loop)

    def _build_ui(self):
        # Apply dark theme styling
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('.', background='#2b2b2b', foreground='#ffffff')
        style.configure('TLabel', background='#2b2b2b', foreground='#ffffff', font=('Helvetica', 10))
        style.configure('TLabelframe', background='#2b2b2b', foreground='#00d084')
        style.configure('TLabelframe.Label', background='#2b2b2b', foreground='#00d084', font=('Helvetica', 10, 'bold'))
        style.configure('TButton', font=('Helvetica', 10, 'bold'))

        # Main Layout: Left Controls, Right Video Panels
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        controls_panel = ttk.Frame(main_frame, width=380, padding=10)
        controls_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        display_panel = ttk.Frame(main_frame, padding=10)
        display_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # -------------------------------------------------------------
        # Left Panel: Slider Controls
        # -------------------------------------------------------------

        # Green Pillar Controls
        green_frame = ttk.LabelFrame(controls_panel, text=" Green Pillar HSV Bounds ", padding=8)
        green_frame.pack(fill=tk.X, pady=(0, 8))

        self.g_h_low = self._create_slider(green_frame, "H Low:", 0, 180, self.detector.GREEN_LOWER[0])
        self.g_h_high = self._create_slider(green_frame, "H High:", 0, 180, self.detector.GREEN_UPPER[0])
        self.g_s_low = self._create_slider(green_frame, "S Low:", 0, 255, self.detector.GREEN_LOWER[1])
        self.g_v_low = self._create_slider(green_frame, "V Low:", 0, 255, self.detector.GREEN_LOWER[2])

        # Red Pillar Controls
        red_frame = ttk.LabelFrame(controls_panel, text=" Red Pillar HSV Bounds ", padding=8)
        red_frame.pack(fill=tk.X, pady=(0, 8))

        self.r_h1_high = self._create_slider(red_frame, "H1 High (0..30):", 0, 30, self.detector.RED_UPPER_1[0])
        self.r_h2_low = self._create_slider(red_frame, "H2 Low (150..180):", 150, 180, self.detector.RED_LOWER_2[0])
        self.r_s_low = self._create_slider(red_frame, "S Low:", 0, 255, self.detector.RED_LOWER_1[1])
        self.r_v_low = self._create_slider(red_frame, "V Low:", 0, 255, self.detector.RED_LOWER_1[2])

        # General Filter Controls
        general_frame = ttk.LabelFrame(controls_panel, text=" Filter & ROI Parameters ", padding=8)
        general_frame.pack(fill=tk.X, pady=(0, 8))

        self.roi_crop = self._create_slider(general_frame, "ROI Top Crop %:", 0, 90, int(self.detector.ROI_TOP_CROP * 100))
        self.min_area = self._create_slider(general_frame, "Min Contour Area:", 10, 3000, self.detector.MIN_CONTOUR_AREA)

        # Action Buttons
        btn_frame = ttk.Frame(controls_panel)
        btn_frame.pack(fill=tk.X, pady=10)

        print_btn = tk.Button(
            btn_frame, text="📋 Print Code Params", bg='#007acc', fg='white',
            font=('Helvetica', 10, 'bold'), relief=tk.RAISED, command=self.print_params
        )
        print_btn.pack(fill=tk.X, pady=4)

        save_btn = tk.Button(
            btn_frame, text="💾 Save to pillar_detector.py", bg='#28a745', fg='white',
            font=('Helvetica', 10, 'bold'), relief=tk.RAISED, command=self.save_params_to_file
        )
        save_btn.pack(fill=tk.X, pady=4)

        # Add an HSV read label below the print/save buttons
        self.hsv_info_label = ttk.Label(btn_frame, text="Click viewfinder to inspect HSV", foreground='#00d084', font=('Helvetica', 10, 'bold'))
        self.hsv_info_label.pack(fill=tk.X, pady=8)

        # -------------------------------------------------------------
        # Right Panel: Video Displays (Viewfinder + Mask)
        # -------------------------------------------------------------

        viewfinder_frame = ttk.LabelFrame(display_panel, text=" Annotated Viewfinder (Detections) ", padding=5)
        viewfinder_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 5))

        self.viewfinder_label = ttk.Label(viewfinder_frame)
        self.viewfinder_label.pack(fill=tk.BOTH, expand=True)
        self.viewfinder_label.bind("<Button-1>", self.on_viewfinder_click)

        mask_frame = ttk.LabelFrame(display_panel, text=" Combined Color Mask (Red ∪ Green) ", padding=5)
        mask_frame.pack(fill=tk.BOTH, expand=True)

        self.mask_label = ttk.Label(mask_frame)
        self.mask_label.pack(fill=tk.BOTH, expand=True)

    def on_viewfinder_click(self, event):
        if not hasattr(self, 'current_hsv'):
            return
        
        # Calculate pixel coordinates in the original unscaled frame
        img_x = event.x - getattr(self, '_vf_offset_x', 0)
        img_y = event.y - getattr(self, '_vf_offset_y', 0)
        
        orig_x = int(img_x * getattr(self, '_vf_scale_x', 1.0))
        orig_y = int(img_y * getattr(self, '_vf_scale_y', 1.0))
        
        h, w = self.current_hsv.shape[:2]
        if 0 <= orig_x < w and 0 <= orig_y < h:
            pixel_hsv = self.current_hsv[orig_y, orig_x]
            self.hsv_info_label.config(text=f"Clicked HSV:  H={pixel_hsv[0]:>3} | S={pixel_hsv[1]:>3} | V={pixel_hsv[2]:>3}")

    def _create_slider(self, parent, label_text, min_val, max_val, default_val):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=2)

        lbl = ttk.Label(frame, text=label_text, width=18, anchor='w')
        lbl.pack(side=tk.LEFT)

        val_label = ttk.Label(frame, text=str(default_val), width=5, anchor='e')
        val_label.pack(side=tk.RIGHT)

        def on_scroll(val):
            val_int = int(float(val))
            val_label.config(text=str(val_int))

        slider = ttk.Scale(
            frame, from_=min_val, to=max_val, value=default_val,
            command=on_scroll
        )
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 4))

        return slider

    def update_loop(self):
        if not self.is_running:
            return

        # Fetch frame
        if self.frame_static is not None:
            frame = self.frame_static.copy()
        elif self.cap is not None:
            ret, frame = self.cap.read()
            if not ret:
                self.root.after(30, self.update_loop)
                return
        else:
            return

        # Read values safely from Tkinter sliders
        self.detector.GREEN_LOWER[0] = int(self.g_h_low.get())
        self.detector.GREEN_UPPER[0] = int(self.g_h_high.get())
        self.detector.GREEN_LOWER[1] = int(self.g_s_low.get())
        self.detector.GREEN_LOWER[2] = int(self.g_v_low.get())

        self.detector.RED_UPPER_1[0] = int(self.r_h1_high.get())
        self.detector.RED_LOWER_2[0] = int(self.r_h2_low.get())
        self.detector.RED_LOWER_1[1] = int(self.r_s_low.get())
        self.detector.RED_LOWER_2[1] = int(self.r_s_low.get())
        self.detector.RED_LOWER_1[2] = int(self.r_v_low.get())
        self.detector.RED_LOWER_2[2] = int(self.r_v_low.get())

        self.detector.ROI_TOP_CROP = self.roi_crop.get() / 100.0
        self.detector.MIN_CONTOUR_AREA = max(10, int(self.min_area.get()))

        # Perform detection
        detections = self.detector.detect(frame)
        annotated = self.detector.draw_detections(frame, detections)

        # Generate combined mask preview
        roi, y_offset = self.detector._preprocess(frame)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Store full frame HSV for click inspector (reconstructing full size)
        self.current_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        g_mask = self.detector._create_green_mask(hsv)
        r_mask = self.detector._create_red_mask(hsv)
        combined_mask = cv2.bitwise_or(g_mask, r_mask)
        mask_bgr = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)

        # Render images in Tkinter canvas
        self._render_to_label(annotated, self.viewfinder_label, max_h=340)
        self._render_to_label(mask_bgr, self.mask_label, max_h=260)

        # Schedule next loop (~30 FPS)
        self.root.after(30, self.update_loop)

    def _render_to_label(self, cv_img, label_widget, max_h=350):
        h, w = cv_img.shape[:2]
        if h == 0 or w == 0:
            return
        aspect = w / h

        widget_h = label_widget.winfo_height()
        if widget_h < 50:
            target_h = max_h
        else:
            target_h = min(max_h, widget_h)

        target_w = max(100, int(target_h * aspect))

        if label_widget is self.viewfinder_label:
            self._vf_scale_x = w / float(target_w)
            self._vf_scale_y = h / float(target_h)
            label_w = label_widget.winfo_width()
            label_h = label_widget.winfo_height()
            self._vf_offset_x = max(0, (label_w - target_w) // 2)
            self._vf_offset_y = max(0, (label_h - target_h) // 2)

        resized = cv2.resize(cv_img, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb)
        img_tk = ImageTk.PhotoImage(image=pil_img)

        label_widget.img_tk = img_tk
        label_widget.config(image=img_tk)

    def print_params(self):
        print("\n=== CURRENT DETECTOR PARAMETERS ===")
        print(f"GREEN_LOWER = np.array({self.detector.GREEN_LOWER.tolist()}, dtype=np.uint8)")
        print(f"GREEN_UPPER = np.array({self.detector.GREEN_UPPER.tolist()}, dtype=np.uint8)")
        print(f"RED_LOWER_1 = np.array({self.detector.RED_LOWER_1.tolist()}, dtype=np.uint8)")
        print(f"RED_UPPER_1 = np.array({self.detector.RED_UPPER_1.tolist()}, dtype=np.uint8)")
        print(f"RED_LOWER_2 = np.array({self.detector.RED_LOWER_2.tolist()}, dtype=np.uint8)")
        print(f"RED_UPPER_2 = np.array({self.detector.RED_UPPER_2.tolist()}, dtype=np.uint8)")
        print(f"ROI_TOP_CROP = {self.detector.ROI_TOP_CROP}")
        print(f"MIN_CONTOUR_AREA = {self.detector.MIN_CONTOUR_AREA}\n")
        messagebox.showinfo("Parameters Printed", "Current parameters printed to terminal console.")

    def save_params_to_file(self):
        detector_path = os.path.join(script_dir, "pillar_detector.py")
        if not os.path.exists(detector_path):
            messagebox.showerror("Error", f"File not found: {detector_path}")
            return

        try:
            with open(detector_path, 'r') as f:
                content = f.read()

            # Update green lower
            import re
            content = re.sub(r'GREEN_LOWER = np\.array\(\[.*?\],', f'GREEN_LOWER = np.array({self.detector.GREEN_LOWER.tolist()},', content)
            content = re.sub(r'GREEN_UPPER = np\.array\(\[.*?\],', f'GREEN_UPPER = np.array({self.detector.GREEN_UPPER.tolist()},', content)
            content = re.sub(r'RED_LOWER_1 = np\.array\(\[.*?\],', f'RED_LOWER_1 = np.array({self.detector.RED_LOWER_1.tolist()},', content)
            content = re.sub(r'RED_UPPER_1 = np\.array\(\[.*?\],', f'RED_UPPER_1 = np.array({self.detector.RED_UPPER_1.tolist()},', content)
            content = re.sub(r'RED_LOWER_2 = np\.array\(\[.*?\],', f'RED_LOWER_2 = np.array({self.detector.RED_LOWER_2.tolist()},', content)
            content = re.sub(r'RED_UPPER_2 = np\.array\(\[.*?\],', f'RED_UPPER_2 = np.array({self.detector.RED_UPPER_2.tolist()},', content)

            with open(detector_path, 'w') as f:
                f.write(content)

            messagebox.showinfo("Success", "Parameters successfully saved to pillar_detector.py!")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save parameters: {str(e)}")

    def on_close(self):
        self.is_running = False
        if self.cap:
            self.cap.release()
        self.root.destroy()


def create_demo_image(width=640, height=480) -> np.ndarray:
    """
    Generates a realistic synthetic camera frame of the WRO 2026 Future Engineers arena
    containing standard Red (RGB: 238, 39, 55) and Green (RGB: 68, 214, 44) traffic pillars.
    """
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # Top half: Background outside track walls (wooden wall / venue floor noise)
    img[0:int(height * 0.42), :] = (140, 130, 120)

    # White track border wall top line
    cv2.rectangle(img, (0, int(height * 0.42)), (width, int(height * 0.45)), (240, 240, 240), -1)

    # Track surface (light matte gray ground)
    img[int(height * 0.45):, :] = (205, 205, 205)

    # Draw Green Pillar (Left side of track, RGB 68, 214, 44 -> BGR 44, 214, 68)
    # Shadow
    cv2.ellipse(img, (180, 410), (45, 15), 0, 0, 360, (150, 150, 150), -1)
    # Front face
    cv2.rectangle(img, (155, 260), (205, 410), (44, 214, 68), -1)
    # Top face (lighter shade)
    pts_top_g = np.array([[155, 260], [175, 245], [225, 245], [205, 260]], np.int32)
    cv2.fillPoly(img, [pts_top_g], (75, 240, 95))
    # Side face (darker shade)
    pts_side_g = np.array([[205, 260], [225, 245], [225, 395], [205, 410]], np.int32)
    cv2.fillPoly(img, [pts_side_g], (30, 175, 50))

    # Draw Red Pillar (Right side of track, RGB 238, 39, 55 -> BGR 55, 39, 238)
    # Shadow
    cv2.ellipse(img, (460, 420), (50, 18), 0, 0, 360, (150, 150, 150), -1)
    # Front face
    cv2.rectangle(img, (430, 250), (490, 420), (55, 39, 238), -1)
    # Top face
    pts_top_r = np.array([[430, 250], [455, 230], [515, 230], [490, 250]], np.int32)
    cv2.fillPoly(img, [pts_top_r], (85, 70, 255))
    # Side face
    pts_side_r = np.array([[490, 250], [515, 230], [515, 400], [490, 420]], np.int32)
    cv2.fillPoly(img, [pts_side_r], (40, 20, 190))

    # Apply slight Gaussian blur to simulate real camera optics
    img = cv2.GaussianBlur(img, (3, 3), 0)
    return img


def main():
    parser = argparse.ArgumentParser(description="WRO 2026 HSV & Contour Threshold Tuning Tool (KDE/Wayland)")
    parser.add_argument("--image", type=str, help="Path to sample image file")
    parser.add_argument("--webcam", type=int, default=None, help="Webcam device ID (e.g., 0)")
    parser.add_argument("--demo", action="store_true", help="Launch GUI with a synthetic WRO 2026 track demo image")
    args = parser.parse_args()

    demo_frame = None
    if args.demo or (args.image is None and args.webcam is None):
        print("Launching in DEMO mode with synthetic WRO 2026 Red and Green pillars...")
        demo_frame = create_demo_image()

    root = tk.Tk()
    app = HSVTuningApp(root, image_path=args.image, webcam_id=args.webcam)
    if demo_frame is not None:
        app.frame_static = demo_frame

    root.mainloop()


if __name__ == "__main__":
    main()
