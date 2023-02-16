#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

int main(int argc, char **argv) {
		
	//read .jpg file
	int w, h, c;
	unsigned char *data = stbi_load(argv[1], &w, &h, &c, 3);
	if (!data) {
        fprintf(stderr, "Cannot load image \"%s\"\nSTB Reason: %s\n", argv[1], stbi_failure_reason());
        exit(0);
    }
		
	//.bin filename
	char * bin_filename = argv[1];
	int filename_len = strlen(argv[1]);
	bin_filename[filename_len-3] = 'b';
	bin_filename[filename_len-2] = 'i';
	bin_filename[filename_len-1] = 'n';
			
	//Open .bin file
	FILE *f_data;
	if ((f_data = fopen(bin_filename, "wb")) == NULL) {
		fprintf(stderr, "unable to open result file\n");
		exit(1);
	}
	
	//Write image dimensions
	fwrite(&w, sizeof(int), 1, f_data);
	fwrite(&h, sizeof(int), 1, f_data);
	fwrite(&c, sizeof(int), 1, f_data);
	
	//Write image pixels
	int i, j, k;
	for(k = 0; k < c; ++k)
        for(j = 0; j < h; ++j)
            for(i = 0; i < w; ++i)
				fwrite(&data[k + c*i + c*w*j], sizeof(unsigned char), 1, f_data);
	
	//close .bin file
	fclose(f_data);
	free(data);
	return 0;
}