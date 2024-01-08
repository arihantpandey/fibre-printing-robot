from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput

net = detectNet("ssd-mobilenet-v2", threshold=0.5)
camera = videoSource("/dev/video0")      # '/dev/video0' for V4L2
display = videoOutput("display://0") # 'my_video.mp4' for file

# Open a text file for writing detections
with open('detections.txt', 'w') as detections_file:
    while display.IsStreaming():
        img = camera.Capture()

        if img is None: # capture timeout
            continue

        detections = net.Detect(img)

        # Write detections to the text file
        for detection in detections:
            detections_file.write(f"Class: {detection.ClassID}, Confidence: {detection.Confidence}, Location: {detection.Left} {detection.Top} {detection.Right} {detection.Bottom}\n")
            bounding_box = (detection.Left, detection.Top, detection.Right, detection.Bottom)

        display.Render(img)
        display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
