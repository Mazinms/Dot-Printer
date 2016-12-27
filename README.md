# Dot-Printer
Using solenoid as a printing end-effector of the Mitsubishi industrial arm rv-2aj to print out RGB images into Dot format images. <br /> Image is first run through edge detector (sobel). Pixel XY locations are converted to function of time. Starting from position Zero following a specific workspace scanning pattern, how much time does it take to reach next pixel with value ONE. <br /> This allows for a faster output and a picture independent system where only workspace edge points are passed to the arm micro-controller. 

# Limitations
Small Ram space to store all the edge XYZ points. Streaming data during RunTime is unreliable, the arm micro-controller response time is not fixed. <br />
