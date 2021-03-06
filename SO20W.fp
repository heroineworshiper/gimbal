	# number of pads
	# pad width in 1/1000 mil
	# pad length in 1/1000 mil
	# pad pitch 1/1000 mil
	# seperation between pads on opposite sides 1/1000 mil
	# X coordinates for the right hand column of pads (1/100 mils)
	# pad clearance to plane layer in 1/100 mil
	# pad soldermask width in 1/100 mil
	# silk screen width (1/100 mils)
	# figure out if we have an even or odd number of pins per side
	# silk bounding box is -XMAX,-YMAX, XMAX,YMAX (1/100 mils)
# element_flags, description, pcb-name, value, mark_x, mark_y,
# text_x, text_y, text_direction, text_scale, text_flags
Element[0x00000000 "Small outline package, wide (300mil)" "" "SO20W" 0 0 -2000 -6000 0 100 0x00000000]
(
# 
# Pad[x1, y1, x2, y2, thickness, clearance, mask, name , pad number, flags]
        Pad[   -30000 -22500 
			 -20000 -22500 
			2000 1000 3000 "1" "1" 0x00000100]
        Pad[   -30000 -17500 
			 -20000 -17500 
			2000 1000 3000 "2" "2" 0x00000100]
        Pad[   -30000 -12500 
			 -20000 -12500 
			2000 1000 3000 "3" "3" 0x00000100]
        Pad[   -30000 -7500 
			 -20000 -7500 
			2000 1000 3000 "4" "4" 0x00000100]
        Pad[   -30000 -2500 
			 -20000 -2500 
			2000 1000 3000 "5" "5" 0x00000100]
        Pad[   -30000 2500 
			 -20000 2500 
			2000 1000 3000 "6" "6" 0x00000100]
        Pad[   -30000 7500 
			 -20000 7500 
			2000 1000 3000 "7" "7" 0x00000100]
        Pad[   -30000 12500 
			 -20000 12500 
			2000 1000 3000 "8" "8" 0x00000100]
        Pad[   -30000 17500 
			 -20000 17500 
			2000 1000 3000 "9" "9" 0x00000100]
        Pad[   -30000 22500 
			 -20000 22500 
			2000 1000 3000 "10" "10" 0x00000100]
        Pad[   30000 22500 
			 20000 22500 
			2000 1000 3000 "11" "11" 0x00000100]
        Pad[   30000 17500 
			 20000 17500 
			2000 1000 3000 "12" "12" 0x00000100]
        Pad[   30000 12500 
			 20000 12500 
			2000 1000 3000 "13" "13" 0x00000100]
        Pad[   30000 7500 
			 20000 7500 
			2000 1000 3000 "14" "14" 0x00000100]
        Pad[   30000 2500 
			 20000 2500 
			2000 1000 3000 "15" "15" 0x00000100]
        Pad[   30000 -2500 
			 20000 -2500 
			2000 1000 3000 "16" "16" 0x00000100]
        Pad[   30000 -7500 
			 20000 -7500 
			2000 1000 3000 "17" "17" 0x00000100]
        Pad[   30000 -12500 
			 20000 -12500 
			2000 1000 3000 "18" "18" 0x00000100]
        Pad[   30000 -17500 
			 20000 -17500 
			2000 1000 3000 "19" "19" 0x00000100]
        Pad[   30000 -22500 
			 20000 -22500 
			2000 1000 3000 "20" "20" 0x00000100]
	ElementLine[-23000 -24500 -23000  24500 1000]
	ElementLine[-23000  24500  23000  24500 1000]
	ElementLine[ 23000  24500  23000 -24500 1000]
	ElementLine[-23000 -24500 -2500 -24500 1000]
	ElementLine[ 23000 -24500  2500 -24500 1000]
	# punt on the arc on small parts as it can cover the pads
	ElementArc[0 -24500 2500 2500 0 180 1000]
)
