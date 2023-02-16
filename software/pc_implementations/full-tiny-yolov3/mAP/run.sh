#clear all existing files
rm *.json
cd ..

#create json file
json_file=mAP/coco_results.json
printf "[\n" >> $json_file

#run makefile
make clean
make FIXED=0 INTERM_DATA=0 LINEAR_EXP=0 GEMM=1 mAP=1 MAX_MIN=1

#run yolo for all images
clear
printf "Running COCO VAL2014 images\n\n"
i=0
for f in ../../new_images/*.bin;
do
	./embedded $f;
	echo [$i] : "${f%.*}" "done"
	((i=i+1))
done

#end json file
sed -i '$ s/.$//' $json_file
printf "]" >> $json_file

#calculate mAP
cd mAP
printf "\n\nCalculating mAP images\n\n"
python calculate_mAP.py