def warp_image(img, angle):
    x = np.radians(-angle)
    y = np.radians(0)
    z = np.radians(180)

    w = img.shape[1]
    h = img.shape[0]
    f = 277 #Focal length 

    dx = 0
    dy= 0
    dz = -w #Translation of image (Distance to camera)

    A1 = np.array([
    [1, 0, -w/2],
    [0, 1, -h/2],
    [0, 0, 0],
    [0, 0, 1]])

    RX = np.array([
    [1, 0,      0,          0],
    [0, cos(x), -sin(x),    0],
    [0, sin(x), cos(x),     0],
    [0, 0,      0,          1]])

    RY = np.array([
    [cos(y), 0, -sin(y), 0],
    [0,      1, 0,       0],
    [sin(y), 0, cos(y),  0],
    [0,      0, 0,       1]])

    RZ = np.array([
    [cos(z), -sin(z), 0, 0],
    [sin(z), cos(z),  0, 0],
    [0,      0,       1, 0],
    [0,      0,       0, 1]])

    rot_mat = RX.dot(RY.dot(RZ))

    T = np.array([
    [1, 0, 0, dx],
    [0, 1, 0, dy],
    [0, 0, 1, dz],
    [0, 0, 0, 1]])

    A2 = np.array([
    [f, 0, w/2, 0],
    [0, f, h/2, 0],
    [0, 0, 1, 0]])

    trans = A2.dot(T.dot(rot_mat.dot(A1)))

    result = cv2.warpPerspective(img, trans, (img.shape[1],img.shape[0]))

    ##Crop image
    h_lines = result.mean(axis = 1)
    w_lines = result.mean(axis = 0)

    for i in range(h_lines.shape[0]):
        if sum(h_lines[i]) > 0:
            min_y = i
            break
    
    for i in range(h_lines.shape[0]-1 , 0,-1):
        if sum(h_lines[i]) > 0:
            max_y = i
            break

    result_cropped = result[min_y:max_y+1, 0:img.shape[1]]

    return result_cropped