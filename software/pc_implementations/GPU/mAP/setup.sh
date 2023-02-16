#!/bin/bash
# Script to setup mAP evaluation
# Execute with command: bash setup.sh

# download dataset
# wget -P $HOME http://images.cocodataset.org/zips/test2017.zip
# unzip dataset
unzip $HOME/test2017.zip -d $HOME

# download txt with image paths to use for test
rm $HOME/testdev2017.txt*
wget -P $HOME https://raw.githubusercontent.com/AlexeyAB/darknet/master/scripts/testdev2017.txt

# update path sufix (from F:/MSCOCO to home/$USER/)
sed -i 's/F:\/MSCOCO/\/home\/'"$USER"'/g' $HOME/testdev2017.txt

# create new folder
rm -rf $HOME/valid_images
mkdir $HOME/valid_images
echo "Created valid_images directory\n"

echo "Copying images for detection from dataset..."
while read line 
do
	#cp file to new folder
	cp $line $HOME/valid_images
done < $HOME/testdev2017.txt
echo "done!"

# Delete test dataset folder«
rm -rf $HOME/test2017

echo "Generating .bin from detection images"
# create bin folder
rm -rf $HOME/new_images
mkdir $HOME/new_images

# Compile .jpg -> .bin converter
make -C ../bin clean
make -C ../bin

# Get total number of images to convert
TOTAL=$(ls $HOME/valid_images/ | wc -l)
i=0

# Convert images to .bin
for f in $HOME/valid_images/*.jpg;
do
	../bin/main $f
	echo "$i / $TOTAL"
	((i=i+1))
done

echo "Copying .bin files to new_images directory..."
# send *.bin to new_image folder
mv $HOME/valid_images/*.bin $HOME/new_images

echo "Setup complete!"
echo "Ready to execute run.sh script"
