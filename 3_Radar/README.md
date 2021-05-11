# 2D CFAR implementation

1. The parameters defined for the:

```Matlab
% Training Cells
Tr = 10;
Td = 8;

% Guard Cells
Gr = 4;
Gd = 4;

% offset the threshold by SNR value in dB
offset = 6;

```

2. Simulation Loop  

* The for loop runs through the image, summing and then averaging it's training cells.  
* We add an offset defined by `offset` variable that adjust the average noise level to a upper boundary.  
* We compare the target cell with the new adjusted boundary and, if  the signal level is lower than the threshold, it is replaced by 0 and if it is greater than the threshold, it replaced by 1. The edge is also surpassed to zero.


```Matlab
for i = Tr+Gr+1:Nr-(Tr+Gr)
    for j = Td+Gd+1:Nd-(Td+Gd)
        ...       
    end
end
```

After performing all the needed operations, the resulting signal, represented in the image below, capture the regions
of interest of our signal.  

<img src="images/3 - CFAR.png" width="779" height="414" />
