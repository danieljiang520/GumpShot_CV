import cv2
import numpy as np
import argparse


def stack_images(scale, img_array, labels=[]):
    """
    Stacks multiple images into one large image, useful for comparing and testing.

    Args:
        scale (float): Scaling factor for resizing the images.
        img_array (list): A list of images (or list of lists of images) to be stacked.
        labels (list, optional): A list of labels corresponding to the images. Defaults to [].

    Returns:
        np.ndarray: The stacked image.
    """
    rows = len(img_array)
    cols = len(img_array[0])
    size_w = img_array[0][0].shape[1]
    size_h = img_array[0][0].shape[0]

    if isinstance(img_array[0], list):
        img_array = [
            [
                cv2.resize(img, (size_w, size_h), None, scale, scale)
                if len(img.shape) != 2
                else cv2.cvtColor(cv2.resize(img, (size_w, size_h)), cv2.COLOR_GRAY2BGR)
                for img in row
            ]
            for row in img_array
        ]
        hor = [np.hstack(row) for row in img_array]
        ver = np.vstack(hor)
    else:
        img_array = [
            cv2.resize(img, (size_w, size_h), None, scale, scale)
            if len(img.shape) != 2
            else cv2.cvtColor(cv2.resize(img, (size_w, size_h)), cv2.COLOR_GRAY2BGR)
            for img in img_array
        ]
        ver = np.hstack(img_array)

    if labels:
        each_img_width = int(ver.shape[1] / cols)
        each_img_height = int(ver.shape[0] / rows)
        for d in range(rows):
            for c in range(cols):
                cv2.rectangle(
                    ver,
                    (c * each_img_width, each_img_height * d),
                    (
                        c * each_img_width + len(labels[d][c]) * 13 + 27,
                        30 + each_img_height * d,
                    ),
                    (255, 255, 255),
                    cv2.FILLED,
                )
                cv2.putText(
                    ver,
                    labels[d][c],
                    (each_img_width * c + 10, each_img_height * d + 20),
                    cv2.FONT_HERSHEY_COMPLEX,
                    0.7,
                    (255, 0, 255),
                    2,
                )
    return ver


def process_image(img, hsv_range):
    """
    Processes the image by converting it to HSV, applying the mask, and detecting circles.

    Args:
        img (np.ndarray): The input image to be processed.
        hsv_range (list): A list of HSV range tuples for masking.

    Returns:
        tuple: A tuple containing the processed image and mask.
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_range[0][0]), np.array(hsv_range[0][1]))

    for x in hsv_range[1:]:
        mask2 = cv2.inRange(hsv, np.array(x[0]), np.array(x[1]))
        mask = cv2.bitwise_or(mask, mask2)

    circles = cv2.HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT,
        1.2,
        img.shape[0],
        None,
        100,
        25,
        int(img.shape[0] / 40),
        int(img.shape[1] / 5),
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (a, b, r) in circles:
            cv2.circle(img, (a, b), r, (255, 0, 0), 7)

    return img, mask


def main(scale, hsv_range):
    """
    Main function to capture video, process images, and display stacked images.

    Args:
        scale (float): Scaling factor for resizing the images.
        hsv_range (list): A list of HSV range tuples for masking.
    """
    cap = cv2.VideoCapture(0)
    print("Capturing...")

    while cap.isOpened():
        ret, img = cap.read()
        if not ret:
            break

        img, mask = process_image(img, hsv_range)
        img_stack = stack_images(scale, ([img], [mask]))
        cv2.imshow('Image', img_stack)

        if cv2.waitKey(27) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("End.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Image stacking and processing with OpenCV.")
    parser.add_argument(
        "--scale", type=float, default=0.3, help="Scale factor for image resizing."
    )
    parser.add_argument(
        "--hsv_range",
        type=list,
        default=[
            [[170, 170, 80], [180, 255, 255]],
            [[0, 170, 80], [10, 255, 255]],
        ],
        help="List of HSV ranges for masking.",
    )
    args = parser.parse_args()

    main(args.scale, args.hsv_range)
