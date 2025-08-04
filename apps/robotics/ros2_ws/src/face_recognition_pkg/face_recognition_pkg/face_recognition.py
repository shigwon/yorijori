from keras_facenet import FaceNet
import cv2
import numpy as np

embedder = FaceNet()
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def crop_face_nearby(img, min_size=100):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)
    big_faces = [f for f in faces if f[2] >= min_size and f[3] >= min_size]
    if len(big_faces) == 0:
        return None
    x, y, w, h = max(big_faces, key=lambda rect: rect[2] * rect[3])
    return img[y:y+h, x:x+w]

def show_result(img1, img2, distance, threshold=0.6):
    h, w = 300, 300
    img1_resized = cv2.resize(img1, (w, h))
    img2_resized = cv2.resize(img2, (w, h))
    text = f"Distance: {distance:.4f} - "
    text += "Same Person" if distance < threshold else "Different Person"
    result_img = np.zeros((h, w*2, 3), dtype=np.uint8)
    result_img[:h, :w] = img1_resized
    result_img[:h, w:] = img2_resized
    cv2.putText(result_img, text, (10, h - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (0, 255, 0) if distance < threshold else (0, 0, 255), 2)
    cv2.imshow("FaceNet Result", result_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    img1 = cv2.imread('face2.jpg')
    img2 = cv2.imread('face5.jpg')
    if img1 is None or img2 is None:
        print("이미지를 불러올 수 없습니다.")
        return

    face1 = crop_face_nearby(img1, min_size=120)
    face2 = crop_face_nearby(img2, min_size=120)
    if face1 is None or face2 is None:
        print("가까운 얼굴을 찾지 못했습니다.")
        return

    emb1 = embedder.embeddings([face1])[0]
    emb2 = embedder.embeddings([face2])[0]

    dist = np.linalg.norm(emb1 - emb2)
    print(f"Distance: {dist:.4f}")
    show_result(face1, face2, dist)

if __name__ == '__main__':
    main()
