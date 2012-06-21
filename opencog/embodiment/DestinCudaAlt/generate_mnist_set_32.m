nR = 32;
nMnist = 28;

nSamples = 10000;

points = [0  0
          0  2
          0  4
          0  6
          0  8
          0  10
          0  12
          0  14
          0  16
          2  14
          4  12
          6  10
          8  8
          10 6
          12 4
          14 2
          16 0
          16 2
          16 4
          16 8
          16 10
          16 12
          16 14
          16 16
         ];

points = points + 1;

mnistFiles = {'mnist_data/train-images-idx3-ubyte', 'mnist_data/train-images-idx3-ubyte', 'mnist_data/test-images-idx3-ubyte'};
outFiles = {'data/destin_train_32.bin', 'data/destin_train_nn_32.bin', 'data/destin_test_nn_32.bin'};
nSamples = [20000 60000 10000];

for f=1:3
	mnistSet = loadMNISTImages(mnistFiles{f})';
	
	fid = fopen(outFiles{f}, 'w');
	
	for d=1:nSamples(f)
	    tmpImage = reshape(mnistSet(d,:), nMnist, nMnist);
	    tmpPadImage = zeros(48,48);
	    
	    tmpPadImage(10:37,10:37) = tmpImage;
	    
	    for i=1:length(points)
	        tmpCropImage = tmpPadImage(points(i,1):points(i,1)+31, points(i,2):points(i,2)+31);
	        fwrite(fid, reshape(tmpCropImage, 1, nR*nR), 'float');
	    end
	end
	
	fclose(fid);
end
