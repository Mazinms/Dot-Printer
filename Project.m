clear all;
clc
%% Picaso and Arduino Uno interface
% Open a serial connection with the Picasso robot
% Close previous serial connections
% Open a serial connection with the Picasso robot
% Close previous serial connections
previous = instrfind('Type','serial');
if(~isempty(previous))
    fclose(previous);
end
% Open a serial connection (default COM1, can be changed)
port = serial('COM4', 'BaudRate', 9600, 'DataBits', 8, 'Parity', 'none','StopBits', 1, 'Timeout',5000, 'Terminator', 13);
delay(0.5);
if (~isempty(port)) fopen(port); delay(0.5);
end
% Turn Servos on
command = sprintf('PRN SVO'); fprintf(port, command); delay(3);
ardi = arduino('COM3','Uno');
delay(3);
soli=11;
limSwitch=10;
writeDigitalPin(ardi,12,1);
writeDigitalPin(ardi,soli,1);
%% Image processing stage
im = imread('logo4.jpg');
[m,n]=size(im)
type sobel;
codegen sobel;
image(im);
gray = (0.2989 * double(im(:,:,1)) + 0.5870 * double(im(:,:,2)) + 0.1140 * double(im(:,:,3)))/255;
edgeIm = sobel_mex(gray, 0.7);
im3 = repmat(edgeIm, [1 1 1]);
im3 = im3>10;
imshow(im3);
BW = im3;
[m,n]=size(BW);

dotMat = resizem(BW,[142 142]);
[m,n] = size(dotMat)
for i=2:2:n
    dotMat(:,i)=zeros(m,1);
end

dotMat([1:4],:)=0;
dotMat([m:-1:m-4],:)=0;
dotMat(:,[1:4])=0;
dotMat(:,[n:-1:n-4])=0;
[row,col] = find(dotMat>0);
k=min(row);
l=max(row);
dotMat=dotMat([k:l],:);
imshow(dotMat)
%% Dot converstion Distance to Time (Method-3)
scanTime=10.45;
dotMatFlp=dotMat;
[m,n] = size(dotMatFlp);
spd1=n/scanTime;
for i=2:2:m
    dotMatFlp(i,:)=fliplr(dotMat(i,:));
end

testMat= dotMat - dotMatFlp;
%imshow(testMat), figure, imshow(dotMat), imshow(dotMatFlp)

timeMat=zeros(n);

for j=1:m
    k=1;
    ip=0;
    for i=1:n
        if dotMatFlp(j,i) == 1
            timeMat(j,k)=d2t(i-ip,spd1)-0.18;
            k=k+1;
            ip=i;
        end
    end
end

totalTime=10.45;
%% Scan routine matrix
% P matrix for the scan routine locations
%position limits Z 240.0 X(218-360) Y(-140+2)
%P [y,x,z]
[m,n]= size(dotMatFlp);
p=[];
for j=1:m*2
    p(j,1)=j;
    if mod(j,2)==0
        p(j,3)=p(j-1,3);
    else
        p(j,3)=-140+(j-1)/2;
    end
    if mod(j,2)==0
        p(j,2)=216+n;
    else
        p(j,2)=218;
    end
    p(j,4)=240;
end

[m,n]=size(p);
A=[218 360 360 218]';
B = repmat(A,1,m);
A=reshape(B,4*m,1);
p(:,2)=A(1:m,1);
%% Picaso points position defining and uploading
%defining all points
[m,n]=size(p);
for i=1:m
    command = sprintf('PRN PD,%1.1f,%1.1f,%1.1f,%1.1f',p(i,1),p(i,2),p(i,3),p(i,4)); %define to positions
    fprintf(port, command);
    m-i
    delay(0.35); %check for minial delay time allowed
end
%% Picaso move to predefinded points & arduino fire solenoid
%moving to defined points
spd2=15;
command = sprintf('PRN SPEED,%1.1f', spd2);
fprintf(port, command);
delay(3);

command = sprintf('PRN MDP,%1.1f', p(1,1));            %move to point
fprintf(port, command);
delay(15);
[m,n]=size(timeMat);
S = sum(timeMat,2);
for j=2:m*2
    command = sprintf('PRN MDP,%1.1f', p(j,1));            %move to point
    fprintf(port, command);
    if mod(j,2)==0
            c=0;
        while c == 0;
            c = readDigitalPin(ardi,limSwitch)
            delay(0.05)
        end
        for i=1:n
            if timeMat((j/2),i)>0;
                t=timeMat(j/2,i);
                delay(t)
                writeDigitalPin(ardi,11,0);
                delay(0.05)
                writeDigitalPin(ardi,11,1);
            end
        end
        delay(scanTime-S(j/2));
    end 
        delay(2)        
    linesfinshed=j
    linesLeft=m-j
end
fclose(port);