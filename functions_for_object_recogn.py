def get_frame_pixel_pos(image):
    ## Get Frame position with respect to the image ###
    ## apply yellow mask
    lowerBound_Y = (0, 100, 100);
    upperBound_Y = (0, 255, 255);
    mask_Y = cv2.inRange(image, lowerBound_Y, upperBound_Y);
    kernel = np.ones((5, 5), np.uint8)
    mask_Y = cv2.dilate(mask_Y, kernel, iterations=3)
    contours, hierarchy = cv2.findContours(mask_Y, 1, 2)
    cnt = contours[0]
    M = cv2.moments(cnt)
    centroid_Y = [M['m10'] / M['m00'], M['m01'] / M['m00']]
    # Plot image for verification

    figure(figsize=(15, 15))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 15))
    ax1.imshow(cv2.cvtColor(mask_Y, cv2.COLOR_BGR2RGB))
    ax2.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    return (centroid_Y)


def get_2nd_joint_pos(image):
    ## Get Frame position with respect to the image ###
    ## apply yellow mask
    lowerBound_B = (100, 0, 0);
    upperBound_B = (255, 0, 0);
    mask_B = cv2.inRange(image, lowerBound_B, upperBound_B);
    kernel = np.ones((5, 5), np.uint8)
    mask_B = cv2.dilate(mask_B, kernel, iterations=3)
    contours, hierarchy = cv2.findContours(mask_B, 1, 2)
    cnt = contours[0]
    M = cv2.moments(cnt)
    centroid_B = [M['m10'] / M['m00'], M['m01'] / M['m00']]
    # Plot image for verification

    figure(figsize=(15, 15))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 15))
    ax1.imshow(cv2.cvtColor(mask_B, cv2.COLOR_BGR2RGB))
    ax2.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    return (centroid_B)


def get_px_meters(pos_center, pos_2nd_joint):
    link_length = 2.5
    ## joint 1 only rotates around z!!
    px_difference = abs(pos_center[2] - pos_2nd_joint[2])
    return link_length / px_difference


def get_orange_pos(image):
    lowerBound_O = (0, 0, 180);
    upperBound_O = (187, 220, 250);
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_O = cv2.inRange(hsv, (10, 100, 20), (25, 255, 255))
    # kernel = np.ones((5, 5), np.uint8)
    # mask_O = cv2.dilate(mask_O, kernel, iterations=3)
    contours, hierarchy = cv2.findContours(mask_O, 1, 2)
    # Plot image for verification
    figure(figsize=(15, 15))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 15))
    ax1.imshow(cv2.cvtColor(mask_O, cv2.COLOR_BGR2RGB))
    ax2.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    return contours


def get_object_features(contours):
    aspect_ratios = []
    rectness = []
    circless = []
    for contour in contours:
        contour_area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        min_rect_area = cv2.minAreaRect(contour)
        min_rect_area = min_rect_area[1][0] * min_rect_area[1][1]
        (x, y), radius = cv2.minEnclosingCircle(contour)
        radius = int(radius)
        aspect_ratios.append(w / h)
        rectness.append(contour_area / min_rect_area)
        circless.append(contour_area / (pi * radius ** 2))
    return (aspect_ratios, rectness, circless)


def recognize_sphere(aspect_ratios, rectness, circless):
    sphericity = [aspect_ratios[idx] - rectness[idx] + circless[idx] for idx in range(len(aspect_ratios))]
    if max(sphericity) < 0.6:
        print('No sphere was detected! Sphere behind Robot!')
        return -1
    else:
        return sphericity.index(max(sphericity))


def get_pos_sphere(contour, sphere_contour_idx):
    cnt = contour[sphere_contour_idx]
    M = cv2.moments(cnt)
    centroid_sphere = [M['m10'] / M['m00'], M['m01'] / M['m00']]
    return centroid_sphere


def detect_sphere_pos(image):
    contours = get_orange_pos(image)
    aspect_ratios, rectness, circless = get_object_features(contours)
    sphere_contour_idx = recognize_sphere(aspect_ratios, rectness, circless)
    if sphere_contour_idx == -1:
        return (0, -1)
    return get_pos_sphere(contours, sphere_contour_idx)

def calculate_pos_sphere(pos_sphere, pos_center, px_to_meters):
    dist = np.sum([(pos_center[i]-pos_sphere[i])**2 for i in range(len(pos_center))])
    dist = px_to_meters*(np.sqrt(dist))
    pos_sphere_meters = [pos_sphere[i] - pos_center[i] for i in range(len(pos_center))]
    return (pos_sphere_meters, dist)



