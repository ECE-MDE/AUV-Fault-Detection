<div align="center">

# AVL Devices Package
</div>


The `avl_devices` package contains low level nodes and drivers that interface with sensors, actuators, and other hardware devices. These nodes may publish sensor information, or provide topics or services that can be used to control or configure the device. The package also provides a `DeviceNode` base class that facilitates the broadcasting of device-specific information using the AVL binary communication protocol's `DEVICE` packet definition.

The devices supported in this package may be used on multiple different vehicles, and any devices that are specific to a single vehicle and will never be used elsewhere should be contained in that vehicle's specific package.

<br/><br/><br/>
<div align="center">

## Device Node Base Class
</div>

Creating a device node that inherits the `DeviceNode` base class allows for the broadcasting of device-specific information using the AVL binary communication protocol's `DEVICE` packet definition. An inheriting class should first call the function
```c++
void set_device_name(std::string name)
```
to set the device name used to identify the device and the function
```c++
void set_device_packet_output_rate(double rate, CommsInterface interface)
```
to set the default output rate for the FSD and BSD interfaces. The node should then implement the
```c++
avl::ParameterList get_device_parameters()
```
function to create and return a parameter list to be broadcast. This function will be called by the device node base class whenever it is time to transmit a `DEVICE` packet to the FSD or BSD interface. The following code snippet demonstrates how to implement the `get_device_parameters()` function.

```c++
avl::ParameterList get_device_parameters()
{

    avl::ParameterList params;
    params.add(Parameter("LAT", position(0)));
    params.add(Parameter("LON", position(1)));
    params.add(Parameter("ALT", position(2)));
    params.add(Parameter("FIX QUALITY", 0));
    params.add(Parameter("NUM SATS",    6));
    return params;

}
```