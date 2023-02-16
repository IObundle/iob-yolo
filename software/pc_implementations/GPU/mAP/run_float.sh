###### Float version
cd ./mAP

#clear all existing files
rm *.json
cd ..

#create json file
json_file=mAP/coco_results.json
printf "[\n" >> $json_file

#compile executable
make clean
make FIXED=0 INTERM_DATA=0 LINEAR_EXP=0 GEMM=1 mAP=1 GPU=1

#run yolo for all images
clear
printf "Running COCO VAL2017 images - Float Version\n\n"

TOTAL=$(ls $HOME/new_images/*.bin | wc -l)
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

mv mAP/coco_results.json detections_test-dev2017_IOBTinyYolov3FloatGPU_results.json

zip detections_test-dev2017_IOBTinyYolov3FloatGPU_results.zip detections_test-dev2017_IOBTinyYolov3FloatGPU_results.json

printf "Zips are ready, scp to local machine and submit for competition\n" 
