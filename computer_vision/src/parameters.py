focal_length = 2.97 #mm
buoy_width = 0.5 #m
buoy_height = 0.5 #m
FOV_x = 80 #^o
FOV_y = 64 #^o

#Yaw_Set:
	#FOV_x / Pixels_x = Degrees/Pixel
	#(Degrees/Pixel) * pixel_width = Degrees

#Distance_Set:
	#(Buoy_width x focal_length) / pixel_width = Distance_z

#Pitch_Set:
	#FOV_y / Pixels_y = Degrees/Pixel
        #(Degrees/Pixel) * pixel_height = Degrees
	#Distance_z * tan^-1(Degrees) = Distance_y
