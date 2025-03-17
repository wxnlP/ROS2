import face_recognition
import cv2
# 通过功能包的名字获取该功能包的安装目录，最终返回安装目录的share目录的绝对路径
from ament_index_python.packages import get_package_share_directory

def main():
    img_path = get_package_share_directory("detect_service_py") + "/resource/pic1.jpg"

    img = cv2.imread(img_path)
    face_location = face_recognition.face_locations(
        img,
        number_of_times_to_upsample=1, 
        model='hog')
    for top, right, bottom, left in face_location:
        cv2.rectangle(img, (left, top), (right, bottom), color=(0, 255, 0), thickness=8)
    cv2.imwrite("/home/sunrise/wkspace1/src/detect_service_py/resource/out2.jpg", img)
    # cv2.imshow("img", img)
    # cv2.waitKey(0)
