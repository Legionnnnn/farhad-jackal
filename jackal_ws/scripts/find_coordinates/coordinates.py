import sys
import cv2
import yaml

name = sys.argv[1].split(".")[0]

with open(f"{name}.yaml", "r") as file:
    data = yaml.safe_load(file)  # safer than yaml.load()

# Load an image (replace with your file path)
img = cv2.imread(f"{name}.pgm")


print(type(img))
print(img.shape)
print(img.dtype)


# img_half = cv2.resize(img, (img.shape[1]//1, img.shape[0]//1))
h, w = img.shape[:2]
img_crop = img[600:h-600, 500:w-500]

resolution = data["resolution"]
print(resolution)

origin = data["origin"]
print(origin)


# Mouse callback function
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse button click
        print(f"Clicked at: x={x}, y={y}")
        # Optional: mark the point on the image
        cv2.circle(img_crop, (x, y), 2, (0, 0, 255), -1)

        cv2.imshow("Image", img_crop)



# Show the image in a window
cv2.imshow("Image", img_crop)

# Set mouse callback
cv2.setMouseCallback("Image", click_event)

# Wait until any key is pressed
cv2.waitKey(0)
cv2.destroyAllWindows()


