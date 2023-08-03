import cv2 as cv2
import numpy as np

def visualize(image, faces, print_flag=False):
    output = image.copy()

    if faces is None:
        return image
    for idx, face in enumerate(faces):
        if print_flag:
            print('Face {}, top-left coordinates: ({:.0f}, {:.0f}), box width: {:.0f}, box height {:.0f}, score: {:.2f}'.format(idx, face[0], face[1], face[2], face[3], face[-1]))

        coords = face[:-1].astype(np.int32)
        # Draw face bounding box
        cv2.rectangle(output, (coords[0], coords[1]), (coords[0]+coords[2], coords[1]+coords[3]), (0, 255, 0), 2)
        # Draw landmarks
        cv2.circle(output, (coords[4], coords[5]), 2, (255, 0, 0), 2)
        cv2.circle(output, (coords[6], coords[7]), 2, (0, 0, 255), 2)
        cv2.circle(output, (coords[8], coords[9]), 2, (0, 255, 0), 2) # nose
        cv2.circle(output, (coords[10], coords[11]), 2, (255, 0, 255), 2)
        cv2.circle(output, (coords[12], coords[13]), 2, (0, 255, 255), 2)
        # Put score
        cv2.putText(output, '{:.4f}'.format(face[-1]), (coords[0], coords[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))

    return output


camera = cv2.VideoCapture(0)

while (True):
    _, img = camera.read()

    model = cv2.FaceDetectorYN.create(
        model="./face_detection_yunet_2022mar.onnx",
        config="",
        input_size=img.shape[:2][::-1],
        top_k=5000
    )


    _, faces = model.detect(img)
    img = visualize(img, faces)
    cv2.imshow("yunet face detection demo", img)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break