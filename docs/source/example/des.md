# CMOS SER/DES generator

![serdes](images/des_1to7_layout.png)

This section describes how to generate CMOS serdes building block layout
templates.

## Deserializer Generation

1) Set your DES parameters in serdes_spec.yaml and serdes_size.yaml
2) Generate 1:N DES

    a) Open serdes_spec.yaml and set num_des and num_flop
    
        num_des = N, num_flop can be 1 or 2, num_flop adjusts the aspect ratio of the layout
        
    b) Generate layout
    
        run laygo/generators/serdes/des_layout_generator.py
        
    c) Generate schematic
    
        run laygo/generators/serdes/des_schematic_generator.py
        

The layout shown below is for 1:32 deserializer.

![serdes](images/des_1to32_layout.png)

You can modify the aspect ratio of the layout by changing num_flop in serdes_spec.yaml.
(num_flop = 1 or 2) 

![serdes16](images/des_1to32_2_layout.png)