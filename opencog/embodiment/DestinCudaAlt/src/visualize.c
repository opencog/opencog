
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#define w 1000
#define h 1000
#define SIZE_LINE 1
#define ARGUMENTS 3
#define READ_FILE_INDEX 1
#define SAVE_FILE_INDEX 2
#define GRAPH_WIDTH 1
#define FRAME_SIZE 0
#define PIXEL_BOX_SIZE 4
#define BUFFER_SIZE 800
#define BYTES_PER_PIXEL 3


//Function prototypes.
void destroyNodeArrays(double** node_array, int size);
int getSize(FILE *file);
double **constructNodeArray(int size);
void getBeliefs(FILE *file, double ** node_array, int size);
int calculateImagePixels(int size);
void destroyColorArrays(double** color_array, int width, int height);
double **constructColorArray(int width, int height);
void fillColorArrays(unsigned char * image, double ** node_array, int size, double ** r, double ** g, double ** b);


int main (int argc, char *argv[]){
  
  
  FILE * read,*write;
  int size;
  double ** node_array;
  
  //Color arrays and pixel array.
  double **r,**g,**b;
  unsigned char * image;
  int width,height;
  
  
  //loop variables
  int i,j;
  
  //Check number of arguments.
  if(argc != ARGUMENTS){
    printf("Incorrect number of arguments. \n Usage:%s inputfile outputfile\n", argv[0]);
    exit(-1);
  }
  
  //Try opening files.
  read = fopen(argv[READ_FILE_INDEX], "r");
  write = fopen(argv[SAVE_FILE_INDEX], "w");
  
  if (read == NULL) {
		printf ("Error: %s cannot be opened, or may not exist. \n", argv[READ_FILE_INDEX]);
		exit (-1);
    
	}
  if (write == NULL) {
		printf ("Error: %s cannot be opened, or may not exist. \n", argv[SAVE_FILE_INDEX]);
    fclose(read);
		exit (-1);
	}
  
  //Get data from file.  First line is number of layers.  Each layer under represents a smaller layer in the heirarchy.
  //Each node on a line is space separated, read from left to right.
  size = getSize(read);
  printf("Size is: %d. Bottom layer will have %d nodes, and layer above it will have %d.\n",
         size, (int)pow(PIXEL_BOX_SIZE, size - 1), (int)pow(PIXEL_BOX_SIZE, size - 2));
  

  //allocate node array.
  node_array = constructNodeArray(size);
  
  //read beliefs from file.
  getBeliefs(read, node_array, size);
  

  //Calculate width and height of the final image.
  width = calculateImagePixels(size);
  printf("width: %d", width);
  //size-1 provides additional room for borders.
  height = width * size + (size-1);
  //printf("width: %d, height: %d \n",width, height);
  
  
  r = constructColorArray(width,height);
  g = constructColorArray(width,height);
  b = constructColorArray(width,height);
  
  image = (unsigned char *)malloc(BYTES_PER_PIXEL*height*width);
    
  //populate color arrays with the node information.
  fillColorArrays(image, node_array, size, r, g, b);
  
  
  //Drawing images.  Should be moved to function.
  int red;
  int green;
  int blue;
  int current_location;
  unsigned char filesize = 54 + 3*width*height;
  
  for(i=0; i<width; i++)
  {
    for(j=0; j<height; j++)
    {
      

      red = r[i][j] * 255;
      green = g[i][j] * 255;
      blue = b[i][j] * 255;
      if (red > 255) red=255;
      if (green > 255) green=255;
      if (blue > 255) blue=255;
      //Test with higher sizes.  If borders are broken, this is where they
      //should be fixed.
      if ((j + 1) % (width + 1) == 0 && j > 1) {red = 200; blue = 10; green = 20;}
      
      printf("(%d,%d) = R:%lf/G:%lf/B:%lf\n",i,j,r[i][j],g[i][j],b[i][j]);
      
      current_location = (i+(j * width)) * 3;
      image[current_location] = (unsigned char)(blue);
      image[current_location+1] = (unsigned char)(green);
      image[current_location+2] = (unsigned char)(red);
    }
  }
  
  unsigned char header[54] = {'B','M',
                                    filesize ,filesize>>8,filesize>>16,filesize>>24,
                                    0,0, 0,0, 54,0,0,0, 40,0,0,0, 
                                    width,width>>8,width>>16,width>>24,
                                    height,height>>8,height>>16,height>>24,
                                    1,0, 24,0};
  unsigned char padding[3] = {0,0,0};
  fwrite(header,1,54,write);
  
  for(i=0; i<height; i++)
  {
    fwrite(image+(width*(height-i-1)*3),3,width,write);
    fwrite(padding,1,(4-(width*3)%4)%4,write);
  }
  
  printf("Done.\n");

  
  //cleanup
  fclose(read);
  fclose(write);
  destroyNodeArrays(node_array, size);
  destroyColorArrays(r, width, height);
  destroyColorArrays(g, width, height);
  destroyColorArrays(b, width, height);
  free(image);
  return 0;
}

//calculates length of width of entire image, with scaled pixel sizes.
int calculateImagePixels(size){
  //Four pixels per node in original image.  Each image should be scaled to same size. ex: 8 true pixels in image
  //on a three layer DeSTIN.
  //# of pixels per a sample(4)^(1/2) will give width and height of sampling box.
  //sampling box times number of nodes in bottom most layer will give true width of original image.
  int width_of_pixel_box = (int)sqrt(PIXEL_BOX_SIZE);
  int number_nodes = (int) pow(PIXEL_BOX_SIZE, size -1);
  int number_nodes_per_row = (int)sqrt(number_nodes);
  int total_pixels_per_row = width_of_pixel_box * number_nodes_per_row;
  //printf("width_of_pixel_box: %d, number_nodes: %d, number_nodes_per_row: %d, total pixels per row: %d\n", width_of_pixel_box, number_nodes, number_nodes_per_row, total_pixels_per_row);
  return total_pixels_per_row;
}


//This should be separated into two functions - one for populating rgb, and one for populating image from rgb.
void fillColorArrays(unsigned char * image, double ** node_array, int size, double ** r, double ** g, double ** b){
  int i,j,k,l,nodes_per_row,layer_box_size,nodes_in_layer,x_offset,y_offset;
  double belief;
  x_offset = y_offset = 0;
  
  //iterate over each layer (0, 1 , 2)
  for(i = 0; i < size; i++){
    nodes_in_layer = pow(PIXEL_BOX_SIZE, (size - i) - 1);
    
    //iterate over number of nodes in that layer. (16,4,1)
    for(j = 0; j < nodes_in_layer; j++){
      //all images should be same size, so size of each drawing box should be scaled.
      nodes_per_row = sqrt(nodes_in_layer);
      layer_box_size = calculateImagePixels(size)/nodes_per_row;
      //printf("Nodes in layer: %d, nodes per row: %d, layer_box_size, %d\n", nodes_in_layer, nodes_per_row, layer_box_size);
      
      //Since we are only working with greyscale currently, set all colors to this value.
      belief = node_array[i][j];
      //printf("Belief: %lf\n", belief);
      
      //calculate offsets
      x_offset = (j % nodes_per_row) * layer_box_size;
      y_offset = (i * calculateImagePixels(size)) + (j / nodes_per_row) * layer_box_size + i;
      
      for (k = x_offset; k < x_offset + layer_box_size; k++) {
        for (l = y_offset; l < y_offset + layer_box_size; l++) {
          //printf("Location x: %d, Location y: %d\n", k , l);
          r[k][l] = belief;
          g[k][l] = belief;
          b[k][l] = belief;
        }
      }
    }
  }
}
                                                                                                                                      

int getSize(FILE *file){
  int size;
  int line = 0;
  char buffer[BUFFER_SIZE +1];
  char *location;
  
  //Keep searching until the size line is found.  Number of commented lines encountered is unpredictable.
  while(line < SIZE_LINE){
    if (fgets(buffer, BUFFER_SIZE, file) == NULL){
      printf("Size could not be gathered.  File not long enough.\n");
      exit(-1);
    }
    location = strchr(buffer, '\n');
    if (!location) {
      printf("Size could not be gathered.  Line %d too large for buffer.\n", line);
      exit(0);
    }
    
    //Set newline to a null terminator.
    *location = '\0';
   
    //Check to make sure line is not a comment.
    location = strchr(buffer, '#');
    if (!location) {
      size = atoi(buffer);
      line++;
    
    }
  }
  return size;
}

void getBeliefs(FILE *file, double ** node_array, int size){
 
  int i, j;
  int result;
  
  for (i = 0; i < size; i++) {
    for(j = 0; j < pow(PIXEL_BOX_SIZE, (size - i) - 1); j++){
      result = fscanf(file, "%lf", &node_array[i][j]); 
      if(!result){
        printf("Error reading beliefs.  File ended without expected number of beliefs reached.\n");
        exit(-1);
        
      }
    }
  }
  
}

/* Mallocs enough space to read in all floats from file for all layers.
 */
double **constructNodeArray(int size){
  double ** return_array = (double **) calloc(size, sizeof(double *));
  int i;
  for (i = size; i > 0; i--) {
    //Each layer (with the lowest layer having the most nodes) has 4^(n-1) nodes.
    return_array[size - i] = (double *) calloc( pow(PIXEL_BOX_SIZE, i - 1), sizeof(double));
  }
  return return_array;
}



void destroyNodeArrays(double** node_array, int size){
  int i;
  for( i = 0; i < size; i++){
    free(node_array[i]);
  }
  free(node_array);
}

double **constructColorArray(int width, int height){
  double ** return_array = (double **) calloc(width, sizeof(double *));
  int i;
  for (i = 0; i < width; i++) {
    return_array[i] = (double *) calloc( height, sizeof(double));
  }
  return return_array;
}


void destroyColorArrays(double** color_array, int width, int height){
  int i;
  for( i = 0; i < width; i++){
    free(color_array[i]);
  }
  free(color_array);
}


                                                                                                                                      
                                                                                                                
