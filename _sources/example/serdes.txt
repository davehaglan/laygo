# SERDES generator

![serdes](images/serdes.png)

This section describes how to generate serdes building block layout
templates.

>Note 1: the codes are lengthy and not cleaned up yet. An improved
 version following the tutorial conventions (possibly with more
abstracting functions) will be released soon.

>Note 2: current version only contains the CMOS serializer part.

## Installation and launch
1. Set up BAG environment for the technology in use. This is usually
done by cloning technology repo provided by BAG developers. For example,
cds_ff_mpt environment can be set up by running following commands.

    ```
    $ git clone git@github.com:ucb-art/BAG2_cds_ff_mpt.git
    $ cd BAG2_cds_ff_mpt
    $ git submodule init
    $ git submodule update
    $ git submodule foreach git pull origin master
    ```

2. Set setting up files, run virtuoso and BAG.

3. Create an empty library called (tech)_logic_templates. For
example, for **cds_ff_mpt** technology, the library name will be
**cds_ff_mpt_logic_template**.

4. Run this command to generate logic layouts.

    ```python
    run laygo/generators/logic/logic_templates_layout_generator.py
    ```

5. Open (tech)_logic_templates library and check if cells are
generated.

    ![logic_lib](images/logic_lib.png)

6. Run this command to generate a serializer layout.

    ```python
    run laygo/generators/serdes/serdes_layout_generator.py
    ```

## Parameterizations

In **laygo/generators/serdes/serdes_layout_generator.py**, there are 2
parameters for serializer, **ser_m**(serializer gate sizing) and
**ser_num_bits**. Users can try different numbers and regenerate the
layout.

The figure shown below is for ser_m=4, ser_num_bits=8.

![serdes](images/serdes_8.png)

The figure shown below is for ser_m=2, ser_num_bits=16.
You can see the number of rows is increased due to the
the increased serialization ratio.

![serdes16](images/serdes_16.png)