import cv2
import numpy as np


def re_combine(img):
    try:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        x_list = []
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, 100, minLineLength=60, maxLineGap=2
        )
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if abs(x1 - x2) < 1:  # Tolerance for verticality
                    x_list.append(x1)
                    # cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        x_list.sort()
        x2 = x_list[-1]
        x1 = x_list[-2]

        height, width = img.shape[:2]

        end_col_x1 = x1
        start_col_x1 = 0

        start_col_x2 = x2
        end_col_x2 = width

        start_row = 0
        end_row = height

        right_side_roi = img[start_row:end_row, start_col_x2:end_col_x2]
        left_side_roi = img[start_row:end_row, start_col_x1:end_col_x1]

        right_side_image = right_side_roi.copy()
        left_side_image = left_side_roi.copy()

        # cv2.imwrite("r.jpg", right_side_image)
        # cv2.imwrite("l.jpg", left_side_image)

        combined_horizontal = cv2.hconcat([right_side_image, left_side_image])
        return combined_horizontal

    except Exception as e:
        print(f"re_combine failed: {e}")
        return None


# img = cv2.imread("output.png")
# cv2.imwrite("comb_hori.jpg", re_combine(img))
