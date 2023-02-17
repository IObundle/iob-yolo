#clear all existing files
clear
make clean

#compile
make

#convert .jpg to .bin
for f in images/*.jpg;
	do ./main $f;
	echo $f "done"
done

#convert .png to .bin
for f in images/*.png;
	do ./main $f;
	echo $f "done"
done

##################################
# COCO VALIDATION SET 2014
##################################

#convert .jpg to .bin
for f in ../../new_images/*.jpg;
	do ./main $f;
	echo $f "done"
done
