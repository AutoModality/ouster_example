#!/usr/bin/env ruby


require 'yaml'
#require 'math'

PIXELS_PER_COLUMN = 64;
COLUMNS_PER_BUFFER = 16
PIXEL_BYTES = 12
COLUMN_BYTES = 16 + (PIXELS_PER_COLUMN * PIXEL_BYTES) + 4;
ENCODER_TICKS_PER_REV = 90112



shape = :SPHERE


def nth_col(n,buf)
  buf[(n * COLUMN_BYTES)..buf.length-1]
end

def col_measurement_id(col_buf)
  col_buf[8..9].pack("C2").unpack("S")[0]
end

def col_timestamp(col_buf)
  col_buf[0..7].pack("C8").unpack("Q")[0]
end

def col_frame_id(col_buf)
  col_buf[10..11].pack("C2").unpack("S")[0]
end

def get_index(col_buf)
  col_buf[10..11].pack("C2").unpack("S")[0]
end

def col_valid(col_buf)
  col_buf[(16 + PIXELS_PER_COLUMN * PIXEL_BYTES)]
end

def nth_px(n, col_buf)
  col_buf[(0 + 16 + n * PIXEL_BYTES)..col_buf.length-1]
end

def px_range(px_buf) 
  px_buf[0..3].pack("C4").unpack("L")[0] & 0x000fffff
end

def set_px_range(px_buf,val)
  val |= 0xfff00000
  px_buf[0..3] = [val].pack("L").unpack("C4")
end

if true

BEAM_ALTITUDE_ANGLES = [
    16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
    12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
    8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
    3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
    -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
    -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
    -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
    -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
];

BEAM_AZIMUTH_ANGLES = [
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
];

IMU_TO_SENSOR_TRANSFORM = [
    1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1]

LIDAR_TO_SENSOR_TRANSFORM = [
    -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1
];

else

BEAM_AZIMUTH_ANGLES = [ 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164 ]

BEAM_ALTITUDE_ANGLES = [ 16.611, 16.084, 15.557, 15.029, 14.502, 13.975, 13.447, 12.92, 12.393, 11.865, 11.338, 10.811, 10.283, 9.756, 9.229, 8.701, 8.174, 7.646, 7.119, 6.592, 6.064, 5.537, 5.01, 4.482, 3.955, 3.428, 2.9, 2.373, 1.846, 1.318, 0.791, 0.264, -0.264, -0.791, -1.318, -1.846, -2.373, -2.9, -3.428, -3.955, -4.482, -5.01, -5.537, -6.064, -6.592, -7.119, -7.646, -8.174, -8.701, -9.229, -9.756, -10.283, -10.811, -11.338, -11.865, -12.393, -12.92, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611 ]

IMU_TO_SENSOR_TRANSFORM= [1.0, 0.0, 0.0, 6.253, 0.0, 1.0, 0.0, -11.775, 0.0, 0.0, 1.0, 7.645, 0.0, 0.0, 0.0, 1.0]
LIDAR_TO_SENSOR_TRANSFORM= [-1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 36.18, 0.0, 0.0, 0.0, 1.0]

end

def make_xyz_lut(w,h,azimuth_angles,altitude_angles)
  n = w * h;
  xyz = Array.new(3 * n,0)
  (0..w-1).each { |icol| 
    h_angle_0 = 2.0 * Math::PI/2 * icol / w;
    (0..h-1).each { |ipx| 
      ind = 3 * (icol * h + ipx);
     
      h_angle = (azimuth_angles[ipx] * 2 * Math::PI / 360.0) + h_angle_0;
              
      xyz[ind + 0] = Math.cos(altitude_angles[ipx] * 2 * Math::PI / 360.0) *
                     Math.cos(h_angle)
      xyz[ind + 1] = -Math.cos(altitude_angles[ipx] * 2 * Math::PI / 360.0) *
                      Math.sin(h_angle)
      xyz[ind + 2] = Math.sin(altitude_angles[ipx] * 2 * Math::PI / 360.0)
    }
  }
  return xyz
end

H = 64
W = 512

xyz_lut = make_xyz_lut(W,H,BEAM_AZIMUTH_ANGLES,BEAM_ALTITUDE_ANGLES)
fp = File.open(ARGV[0])
fp.readlines.each { |d|
  next if d =~ /^---/
  buf = YAML.load(d)["buf"]
  (0..COLUMNS_PER_BUFFER-1).each { |icol|
    # byebug
    col_buf = nth_col(icol, buf)
    m_id = col_measurement_id(col_buf);
    f_id = col_frame_id(col_buf);
    ts = col_timestamp(col_buf);
    valid = col_valid(col_buf) == 0xffffffff;
    STDERR.puts "M_id(#{m_id}) F_ID(#{f_id})"
    # const uint8_t* px_buf = OS1::nth_px(ipx, col_buf);
    # uint32_t r = OS1::px_range(px_buf);
    # r = 1000;
    # int ind = 3 * (idx + ipx);
    byebug
    idx = H * m_id;
    (0..H-1).each { |ipx|
      px_buf = nth_px(ipx, col_buf);
      r      = px_range(px_buf);
      ind    = 3*(idx+ipx)
      (r * 0.001 * xyz_lut[ind + 0],r * 0.001 * xyz_lut[ind + 1],r * 0.001 * xyz_lut[ind + 2])
    }

    if shape == :SPHERE
      
    else

    end

  }
  # byebug
  puts ""
}


# buf[10..11].pack("C2").unpack("S")[0]
