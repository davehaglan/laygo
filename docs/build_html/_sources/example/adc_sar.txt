# Time-interleaved SAR ADC generator

This section describes how to generate a time-interleaved SAR ADC
schematic and layout using BAG and laygo.

## Overview

Time Interleaved Successive Approximation ADC (TISARADC) is a popular
way of implementing a moderate precision (4-12 bits) ADC that operates
at higher sampling rates than single SAR ADCs can achieve. As an example
 of demonstrating the automated analog and mixed signal circuits
 generation flow, the entire TISARADC generator is explained here.

## Installation

1. Set up a BAG/laygo working directory for the your technology.
For example, cds_ff_mpt environment can be set up by running the
following commands.

    ```
    $ git clone git@github.com:ucb-art/BAG2_cds_ff_mpt.git
    $ cd BAG2_cds_ff_mpt
    $ git submodule init
    $ git submodule update
    $ git submodule foreach git pull origin master
    ```

    > Note: for BWRC users, comprehensive working environments are
    provided under proper NDAs for certain technologies. Use the
    following repos.

    > 1. **tsmcN16**: [git@bwrcrepo.eecs.berkeley.edu:jdhan/TISARADC_TSMC16FFC.git](git@bwrcrepo.eecs.berkeley.edu:jdhan/TISARADC_TSMC16FFC.git)
    > 2. **tsmcN28**:

2. Run virtuoso and BAG.

3. In the IPython interpreter, type the following command to construct
the template and grid database.

    ```
    run laygo/labs/lab2_a_gridlayoutgenerator_constructtemplate.py
    ```

4. Run the following command to generate the logic gate template
library. The library name will be (technology_name)_logic_templates.

    ```
    run laygo/generators/logic/logic_templates_layout_generator.py
    ```

## Setting up parameters

ADC schematic and layout are constructed based on spec/size parameters
defined in 2 configuration files in **YAML** format
(**adc_sar_spec.yaml, adc_sar_size.yaml**).

Following parameters are defined in the configuration files:

1. adc_sar_spec.yaml

    * **temp**: nominal temperature
    * **temp_min**: minimum temperature
    * **temp_max**: maximum temperature
    * **v_in**: nominal input swing (single ended)
    * **v_in_max**: maximum input swing
    * **v_in_cm**: nominal inpuut common mode
    * **n_bit**: number of output bits
    * **n_bit_cal**: number of output bits after digital calibration
    * **fsamp**: effective sampling rate
    * **n_interleave**: interleaving ratio
    * **c_unit**: minimum unit capacitance of CDAC
    * **c_ground**: grounded parasitic capacitance of c_unit
    * **c_delta**: maximum capacitance mismatch of c_unit (normalized by
    c_unit)
    * **n_bit_samp_noise**: standard deviation of sampling noise (in bits)
    * **n_bit_samp_settle**: maximum sampler settling error in bits
    * **n_bit_comp_noise**: standard deviation of comparator noise in bits
    * **rdx_array**: CDAC radix array
    * **rdx_enob**: Target ENOB after calibration
    * **cycle_comp**: Comparator timing budget in cycle
    * **cycle_logic**: Logic timing budget in cycle
    * **cycle_dac**: DAC timing budget in cycle
    * **vdd**: nominal supply voltage
    * **vdd_min**: minimum supply voltage
    * **vdd_max**: maximum supply voltage

2. adc_sar_size.yaml
    * **capdac_c_m**: multiplier of CDAC unit capacitor
    * **capdac_num_bits_vertical**: CDAC number of bits in vertical
    direction
    * **capdac_num_bits_horizontal**: CDAC number of bits in horizontal
    direction
    * **capdrv_m_list**: cap driver multiplier list
    * **capdrv_space_offset**: cap driver spacing parameter, for routing
    * **salatch_m**: strongArm latch sizing
    * **salatch_m_buf**: strongArm latch output buffer sizing
    * **salatch_m_rgnn**: strongArm latch regenerative latch sizing
    * **salatch_m_rst**: strongArm latch reset device sizing
    * **sarclkgen_m**: strongArm latch clock generator sizing
    * **sarclkgen_fo**: strongArm latch clock generator fanout
    * **sarret_m**: strongArm latch output retimer sizing
    * **sarret_fo**: strongArm latch output retimer fanout
    * **sarfsm_m**: SAR FSM sizing
    * **sarlogic_m**: SAR logic sizing
    * **dcap2_m**:
    * **num_space_samp**:
    * **num_space_sar**:

For the starting point, following preset files are provided. They are
not 100% finalized, but will serve well for initializations.

* **adc_sar_spec(size)_craft.yaml** : 9.6GS/s, 9bit ADC with digital
calibration.
* **adc_sar_spec(size)_11b.yaml** : GS/s, 11bit ADC with digital
calibration.
* **adc_sar_spec(size)_6b.yaml** : 2GS/s, 6bit ADC with radix-2
constellation.

## TISARADC architecture

TISARADC is composed of 4 parts in the top level: ADC core, clock
generator, output retimer and biasing.

* The **ADC core** performs the actual conversion operation. It is
composed of multi-phased sampling frontend and N subADC that
converts the actual ADC operation at (sampling_rate/N), where N is
the interleaving ratio.
* The **clock generator** receives the high frequency clock at sampling
rate, generates multi-phased clock for the interleaved ADC operation.
* The **output retimer** receives the output from the ADC core, aligns
multi-phased signals to a single clock to be received by the
following digital backend.

## SubADC frontend layout generation

The frontend of subADCs is composed of three parts: capdac (CDAC),
strongArm comparator (salatch), cap driver (capdrv). In the proposed
flow, the 'very' frontend sampling switches are not the part of
 subADC frontend because they are designed separately, but obviously
 this is not the only way of partition.

1. The first block to be generated is CDAC, since this generator
assumes the horizontal dimension is limited by CDAC size in x-direction.
In order to generate CDAC layout, type:

    ```
    run laygo/generators/adc_sar/adc_sar_capdac_layout_generator.py
    ```

    This command will create a CDAC layout in
    **adc_sar_generated/capdac**.

    * If you want to generate the schematic as well, type:

        ```
        run laygo/generators/adc_sar/adc_sar_capdac_schematic_generator.py
        ```

    * The schematic generation is not required for full-ADC
    generations, since you can generate the whole ADC schematic by
    running a top-cell generation script (unlike the layout counterpart,
    which requires running several scripts in series to generate the
    entire ADC). The CDAC schematic generation is used for cell level
    verifications though, such as LVS or transfer curve extraction. For
    LVS, type:

        ```
        run laygo/generators/adc_sar/adc_sar_capdac_lvs.py
        ```

        It will give a **lvs passed**  message if the design is LVS
        clean.

    * For parasitic extraction, type the following command.

        ```
        run laygo/generators/adc_sar/adc_sar_capdac_extract.py
        ```

    * You can run simulations on CDAC via BAG framework. The following
    command will launch the transfer curve extraction simulation with
    schematic netlists. For the post-extract simulation, set
    **extracted = True** in the script.

        ```
        run laygo/generators/adc_sar/adc_sar_capdac_verify.py
        ```

2. After CDAC generation, strongArm latch layout can be generated by
running:

    ```
    run laygo/generators/adc_sar/adc_sar_salatch_pmos_layout_generator.py
    ```

    * Here are additional scripts regarding the strongArm latch:

        1. **adc_sar_salatch_pmos_schematic_generator.py**: schematic
        generation
        2. **adc_sar_salatch_pmos_schematic_lvs.py**: LVS
        3. **adc_sar_salatch_pmos_schematic_extract.py**: RC extraction
        4. **adc_sar_salatch_pmos_verify.py**: verifications (clk-q,
        noise)

3. The next step is generating capacitor drivers that drive CDAC. Run
the following scripts:

    ```
    run laygo/generators/adc_sar/adc_sar_capdrv_nsw_layout_generator.py
    run laygo/generators/adc_sar/adc_sar_capdrv_nsw_array_layout_generator.py
    ```

    This is 2-step generation; in the first script, individual driver
    cells are generated. The second script places driver cells (with
    the sizing parameters defined by **capdrv_m_list** in
    **adc_sar_size.yaml**.

4. Now it's time to generate the frontend top layout/schematic. Type:

    ```
    run laygo/generators/adc_sar/adc_sar_sarafe_nsw_layout_generator.py
    run laygo/generators/adc_sar/adc_sar_sarafe_nsw_schematic_generator.py
    run laygo/generators/adc_sar/adc_sar_sarafe_nsw_lvs.py
    ```

    The commands will create a layout (and schematic) of sarafe_nsw,
    which is the frontend top of subADC, and run LVS. Try different
    presets introduced before and see if designs are generated properly.

## SubADC backend layout generation

## SubADC switch generation

## Clocking path generation

## Retimer

## Bias

## Top


## Contributors

The generator is written by Jaeduk Han, Eric Chang, Zhongkai Wang,
Woorham Bae, and Pengpeng Lu. This document is written by Jaeduk Han.