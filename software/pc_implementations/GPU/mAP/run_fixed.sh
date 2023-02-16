#FIXED version

#clear all existing files
rm *.json
cd ..

#create json file
json_file=mAP/coco_results.json
printf "[\n" >> $json_file

#compile executable
make clean
make FIXED=1 INTERM_DATA=0 LINEAR_EXP=1 GEMM=0 mAP=1 GPU=1

#run yolo for all images
clear
printf "Running COCO VAL2017 images - Fixed version\n\n"

#TOTAL=$(ls $HOME/new_images/*.bin | wc -l)
i=0
for f in $HOME/new_images/*.bin;
do
	./embedded $f;
	echo [$i / $TOTAL] : "${f%.*}" "done"
	((i=i+1))
done

#end json file
sed -i '$ s/.$//' $json_file
printf "]" >> $json_file

printf "\n\nCalculated all results\n"

mv mAP/coco_results.json detections_test-dev2017_IOBTinyYolov3FixedGPU_results.json

zip detections_test-dev2017_IOBTinyYolov3FixedGPU_results.zip detections_test-dev2017_IOBTinyYolov3FixedGPU_results.json
