def generate_arg_string(input_string, _id, rate):
    values = input_string.split(',')
    labels = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    output_string = '    <include file="$(find oem_client)/launch/client.launch">\n'

    output_string += f'        <arg name="namespace" value="v{_id}" />\n'

    for label, value in zip(labels, values):
        output_string += f'        <arg name="{label}" value="{value.strip()}" />\n'

    output_string += f'        <arg name="bagfile" value="$(env DATA_ROOT)hdl-64s2/segment/segment_segment_{(_id-1)*100}_{_id*100}_0.bag" />\n'
    output_string += f'        <arg name="rate" value="{rate}" />"\n'
    output_string += '    </include>\n'
    return output_string


input_string = ["-392.593,-304.418,5.14688,0.0156697,-0.05112,-0.375823",
                "-70.5268,-394.231,11.1009,-0.0359059,0.0417539,0.529804",
                "283.11,-612.964,30.9785,-0.0184782,-0.073197,-1.23474",
                "393.692,-917.103,30.9025,-0.0237223,0.0352151,-1.38803",
                "706.397,-848.401,18.8839,-0.0147032,-0.038648,0.42599"]

for i in range(6, 11):
    print(generate_arg_string(input_string[i-6], i, 1.0))
