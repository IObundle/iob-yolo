#clear all existing files
clear
make clean
filename=dog.bin

#check if output folder exists and clean it
rm -rf ../output_data/*
mkdir -p ../output_data/

#update IMG_W and IMG_H of embedded.h file

#compile
make FIXED=1 INTERM_DATA=0 LINEAR_EXP=0 GEMM=1 GPU=0

#object detection
./embedded data/$filename
#gprof embedded gmon.out > embedded.txt

#write output image
python python/write_image.py ../output_data/$filename
