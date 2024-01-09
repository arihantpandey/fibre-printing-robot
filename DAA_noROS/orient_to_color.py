import cv2
import numpy as np

# Define the color that needs to be recognized.

#Yellow #FFFF00
# colorUpper = np.array([44, 255, 255])
# colorLower = np.array([24, 100, 100])

# # Red FF0000
colorUpper = np.array([180, 255, 255])
colorLower = np.array([160, 100, 100])

# Green #00FF00
# colorUpper = np.array([50, 255, 255])
# colorLower = np.array([70, 200, 100])

# Blue #0000FF
# colorUpper = np.array([110, 225, 255])
# colorLower = np.array([135, 180, 200])

# Cyan #00FFFF
# colorUpper = np.array([80, 255, 255])
# colorLower = np.array([105, 180, 180])

# Magenta #FF00FF
# colorUpper = np.array([140, 255, 255])
# colorLower = np.array([160, 150, 200])

error_tor = 25
PID_P = 3

# Color recognition and tracking function.
def centerTargetColor(imageInput):
    # Convert video frames to HSV color space.
    hsv = cv2.cvtColor(imageInput, cv2.COLOR_BGR2HSV)
    
    # Create a mask for pixels that match the target color.
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    
    # Erode, this process will remove the relatively 
    # small area in the mask just selected, which can be understood as denoising.
    mask = cv2.erode(mask, None, iterations=2)
    
    # dilate, the corrosion process just now will cause the large area to become 
    # smaller and the small area to disappear. This step is to restore the large area to its previous size.
    mask = cv2.dilate(mask, None, iterations=2)
    
    # Obtain the conformed area contour.
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    height, width = imageInput.shape[:2]  # Assuming imageInput is a numpy array representing the image
    center_x, center_y = width // 2, height // 2
    
    # If there is a matching area, start to control the movement of the steering gear to achieve color tracking.
    if len(cnts) > 0:
        # Draw text to show that the target has been found.
        imageInput = cv2.putText(imageInput,'Target Detected',(10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
        
        # Find the contour of the largest area.
        c = max(cnts, key=cv2.contourArea)

        ((box_x, box_y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        
        # X, Y are the center points of the area.
        X = int(box_x)
        Y = int(box_y)
        
        error_Y = abs(150 - Y)
        error_X = abs(150 - X)
        
        # Draw the size and position of this area.
        cv2.rectangle(imageInput,(int(box_x-radius),int(box_y+radius)),(int(box_x+radius),int(box_y-radius)),(255,255,255),1)
        
        if Y < center_y - error_tor:
            imageInput = cv2.putText(imageInput,'LOOK UP',(10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
            
        elif Y > center_y + error_tor:
            imageInput = cv2.putText(imageInput,'LOOK DOWN',(10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)

        else:
            imageInput = cv2.putText(imageInput,'Y AXIS LOCKED',(10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)


        if X < center_x - error_tor:
            imageInput = cv2.putText(imageInput,'LOOK LEFT',(10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)

        elif X > center_x + error_tor:
            imageInput = cv2.putText(imageInput,'LOOK RIGHT',(10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)

        else:
            imageInput = cv2.putText(imageInput,'X AXIS LOCKED',(10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)

    else:
        imageInput = cv2.putText(imageInput,'Target Detecting',(10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
    
    return imageInput


cap = cv2.VideoCapture("/dev/video0")


cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)

cap_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
cap_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
cap_fps = cap.get(cv2.CAP_PROP_FPS)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    
    # cap_fps = cap.get(cv2.CAP_PROP_FPS)
    # print("FPS: " + str(cap_fps), end="\r", flush=True)
    
    # Process the frame with the findColor function
    annotated_frame = centerTargetColor(frame)
    
    # Display the processed frame
    cv2.imshow('Video Stream', annotated_frame)
    
    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture and destroy all windows.
cap.release()
cv2.destroyAllWindows()
