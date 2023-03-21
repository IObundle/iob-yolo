#clear all existing files
#clear
make clean
filename=dog.bin

#check if output folder exists and clean it
rm -rf ../output_data/*
mkdir -p ../output_data/
rm -rf ../debug_data/*
mkdir -p ../debug_data/

#delete hex output files
rm ../yolov3-tiny_*_conv.hex

#compile floating point version
make EIGHT_BITS=0 FIXED=0 INTERM_DATA=0 LINEAR_EXP=0 GEMM=0 MAX_MIN=0
#object detection
./embedded data/$filename > ../debug_data/embedded_float.txt
#gprof embedded gmon.out > embedded.txt

make clean
#compile 8-bit version
make EIGHT_BITS=1 FIXED=0 INTERM_DATA=0 LINEAR_EXP=0 GEMM=0 MAX_MIN=0
#object detection
./embedded data/$filename > ../debug_data/embedded_8-bit.txt
#gprof embedded gmon.out > embedded.txt

#compute error
python3 python/compute_error.py > 8b_err_out.txt
