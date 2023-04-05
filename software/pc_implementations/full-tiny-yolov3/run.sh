#clear all existing files
#clear
make clean
filename=dog.bin

#check if output folder exists and clean it
rm -rf ../output_data/*
mkdir -p ../output_data/

#compile
make EIGHT_BITS=1 FIXED=0 INTERM_DATA=0 LINEAR_EXP=0 GEMM=0 MAX_MIN=0

#object detection
./embedded data/$filename
#gprof embedded gmon.out > embedded.txt

#write output image
python3 python/write_image.py ../output_data/$filename
