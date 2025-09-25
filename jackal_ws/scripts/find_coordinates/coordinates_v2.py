import sys
import cv2
import yaml

name = sys.argv[1].split(".")[0]

# Load YAML
with open(f"{name}.yaml", "r") as file:
    data = yaml.safe_load(file)

# Load map image
img = cv2.imread(f"{name}.pgm")
h, w = img.shape[:2]

# --- Cropping offsets (change these as you like) ---
crop_top = 800
crop_bottom = 500
crop_left = 500
crop_right = 400

# Apply cropping
img_crop = img[crop_top:h-crop_bottom, crop_left:w-crop_right]

# Get map info
resolution = data["resolution"]  # meters per pixel
origin = data["origin"]          # [x0, y0, theta]

print("Resolution:", resolution)
print("Origin:", origin)
print("Image size:", w, "x", h)
print("Cropped size:", img_crop.shape[1], "x", img_crop.shape[0])

# Mouse callback
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  
        # Convert cropped coordinates back to full image coordinates
        x_full = x + crop_left
        y_full = y + crop_top

        # Convert to world coordinates
        world_x = origin[0] + x_full * resolution
        world_y = origin[1] + (h - y_full) * resolution  # flip y-axis

        print(f"Pixel clicked (cropped): (x={x}, y={y})")
        print(f"Pixel in full image: (x={x_full}, y={y_full})")
        print(f"World coord: (X={world_x:.2f}, Y={world_y:.2f})")

        # Mark on cropped image
        cv2.circle(img_crop, (x, y), 2, (0, 0, 255), -1)
        cv2.imshow("Image", img_crop)

# Show cropped image
cv2.imshow("Image", img_crop)
cv2.setMouseCallback("Image", click_event)
cv2.waitKey(0)
cv2.destroyAllWindows()