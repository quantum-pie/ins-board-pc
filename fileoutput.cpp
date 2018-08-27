#include "fileoutput.h"
#include "packets.h"
#include "utils.h"

FileOutput::FileOutput() : RunningFlag{ false } {}

void FileOutput::handle_input(const RawPacket & z)
{
    if(is_running())
    {
        if(output_file.is_open() && output_file.tellp() < batch_size)
        {
            output_file.write(reinterpret_cast<const char*>(&z.ref_pitch), sizeof(z.ref_pitch));
            output_file.write(reinterpret_cast<const char*>(&z.ref_roll), sizeof(z.ref_roll));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.time.year), sizeof(z.gps_data.time.year));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.time.month), sizeof(z.gps_data.time.month));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.time.day), sizeof(z.gps_data.time.day));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.time.hour), sizeof(z.gps_data.time.hour));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.time.minute), sizeof(z.gps_data.time.minute));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.time.second), sizeof(z.gps_data.time.second));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.time.ssecond), sizeof(z.gps_data.time.ssecond));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.geo[0]), sizeof(z.gps_data.geo[0]));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.geo[1]), sizeof(z.gps_data.geo[1]));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.geo[2]), sizeof(z.gps_data.geo[2]));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.msl_altitude), sizeof(z.gps_data.msl_altitude));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.pos[0]), sizeof(z.gps_data.pos[0]));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.pos[1]), sizeof(z.gps_data.pos[1]));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.pos[2]), sizeof(z.gps_data.pos[2]));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.vel[0]), sizeof(z.gps_data.vel[0]));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.vel[1]), sizeof(z.gps_data.vel[1]));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.vel[2]), sizeof(z.gps_data.vel[2]));
            output_file.write(reinterpret_cast<const char*>(&z.gps_data.fix), sizeof(z.gps_data.fix));
            output_file.write(reinterpret_cast<const char*>(&z.w[0]), sizeof(z.w[0]));
            output_file.write(reinterpret_cast<const char*>(&z.w[1]), sizeof(z.w[1]));
            output_file.write(reinterpret_cast<const char*>(&z.w[2]), sizeof(z.w[2]));
            output_file.write(reinterpret_cast<const char*>(&z.a[0]), sizeof(z.a[0]));
            output_file.write(reinterpret_cast<const char*>(&z.a[1]), sizeof(z.a[1]));
            output_file.write(reinterpret_cast<const char*>(&z.a[2]), sizeof(z.a[2]));
            output_file.write(reinterpret_cast<const char*>(&z.m[0]), sizeof(z.m[0]));
            output_file.write(reinterpret_cast<const char*>(&z.m[1]), sizeof(z.m[1]));
            output_file.write(reinterpret_cast<const char*>(&z.m[2]), sizeof(z.m[2]));
            output_file.write(reinterpret_cast<const char*>(&z.et), sizeof(z.et));
        }
        else if(z.gps_data.fix)
        {
            output_file.close();
            output_file.open("res/raw_" + utils::time_string(z.gps_data.time) + ".dat", std::ios::binary);
        }
    }
}

void FileOutput::handle_enable(bool en)
{
    set_running(en);
    if(!en)
    {
        output_file.close();
    }
}
