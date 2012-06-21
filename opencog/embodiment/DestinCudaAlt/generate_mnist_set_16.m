nR = 16;
nMnist = 28;

nSamples = 60000;

points = [0  0
          0  2
          0  4
          0  6
          0  8
          0  10
          0  12
          2  10
          4  8
          6  6
          8  4
          10 2
          12 0
          12 2
          12 4
          12 6
          12 8
          12 10
          12 12
         ];

points = points + 1;

mnistFiles = {'mnist_data/train-images-idx3-ubyte', 'mnist_data/train-images-idx3-ubyte', 'mnist_data/test-images-idx3-ubyte'};
outFiles = {'data/destin_train_16.bin', 'data/destin_train_nn_16.bin', 'data/destin_test_nn_16.bin'};
nSamples = [20000 60000 10000];

for f=1:3
	mnistSet = loadMNISTImages(mnistFiles{f})';
	
	fid = fopen(outFiles{f}, 'w');
	
	for d=1:nSamples
	    tmpImage = reshape(mnistSet(d,:), nMnist, nMnist);
	    
	    for i=1:length(points)
	        tmpCropImage = tmpImage(points(i,1):points(i,1)+15, points(i,2):points(i,2)+15);
	        fwrite(fid, reshape(tmpCropImage, 1, nR*nR), 'float');
	    end
	end
	
	fclose(fid);
end
