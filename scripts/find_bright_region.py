import cv2
import numpy as np
import argparse
import os

# this seriously sucks but it works...
def find_bright_region(image_path, output_path, blur_size=(11, 11), threshold_ratio=0.9):
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not read image at {image_path}")
        return False

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(gray, blur_size, 0)

    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(blurred)

    threshold_val = max_val * threshold_ratio
    _, mask = cv2.threshold(blurred, threshold_val, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        print("No bright regions found.")
        return False

    largest_contour = max(contours, key=cv2.contourArea)

    x, y, w, h = cv2.boundingRect(largest_contour)

    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 31, 63), 20)

    label = f"BRIGHT REGION ({int(max_val)})"
    font = cv2.FONT_HERSHEY_SIMPLEX

    font_scale = max(0.2, img.shape[1] / 1000.0)
    font_thickness = max(2, int(font_scale * 2))

    (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, font_thickness)

    cv2.rectangle(img, (x - 10, y - text_h - 10), (x + text_w + 10, y), (0, 31, 63), -1)

    cv2.putText(img, label, (x - 10, y - 10), font, font_scale, (255, 255, 255), font_thickness)

    # Save output
    cv2.imwrite(output_path, img)
    print(f"Success: Output saved to {output_path}")
    print(f"Brightest point at: {max_loc}, Max Brightness: {max_val}")
    print(f"Region Bounding Box: x={x}, y={y}, w={w}, h={h}")

    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Find the brightest region in an image.")
    parser.add_argument("--input", required=True, help="Path to input image")
    parser.add_argument("--output", default="bright_region_output.png", help="Path to output image")
    parser.add_argument("--threshold", type=float, default=0.9, help="Threshold ratio (0.0 to 1.0) of max brightness")

    args = parser.parse_args()

    if os.path.exists(args.input):
        find_bright_region(args.input, args.output, threshold_ratio=args.threshold)
    else:
        print(f"Input file not found: {args.input}")
