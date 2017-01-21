--[[
 ADS1015 library for NodeMCU
 Requires tmr, bit, i2c
 Default address: 0x48
 Default SDA:     3
 Default SCL:     4
 
 Differential comparison only possible between
 channels 0 & 1, 0 & 3, 1 & 3, 2 & 3.
 
 Example:
 
 ads1015 = dofile("ads1015.lua")(3, 4, 0x48)
 ads1015.gain = ads1015.GAIN_ONE
 print("\n", ads1015:read(0))
 print("\n", ads1015:read(0, 1)) -- differential between 0 and 1
 
 ads1015:comparator_start(200, 1000, 0, nil, bit.bor(
 	ads1015._cque_4conv,   
     ads1015._clat_nonlat,    
     ads1015._cpol_actvlow,  
     ads1015._cmode_window,    
     ads1015._sps_1600,    
     ads1015._mode_contin
 ))
 print("\n", ads1015:read_last_conversion())
]]

local write_register = function(self, reg, value)
	i2c.start(0)
	i2c.address(0, self._adr, i2c.TRANSMITTER)
	i2c.write(0, reg, bit.rshift(value, 8), bit.band(value, 0xFF))
	i2c.stop(0)
end

local read_register = function(self)
	i2c.start(0)
	i2c.address(0, self._adr, i2c.TRANSMITTER)
	i2c.write(0, 0)
	i2c.stop(0)
	i2c.start(0)
	i2c.address(0, self._adr, i2c.RECEIVER)
	value = i2c.read(0, 2)
	i2c.stop(0)	
	value = bit.rshift(bit.bor(bit.lshift(value:byte(1), 8), value:byte(2)), 4)
	
	-- keep negativity
	if value > 0x07ff then
		value = bit.set(bit.bor(value, 0x7FFFF000), 31)
	end
	return value
end

local calc_mux = function(self, channel1, channel2)
	if channel1 > 3 or (channel2 and channel2 > 3) then
		print("\nChannel out of bounds.")
		do return end
	end
	
	if channel2 == nil then
		if channel1 == 0 then
			return self._mux_single_0
		elseif channel1 == 1 then
			return self._mux_single_1
		elseif channel1 == 2 then
			return self._mux_single_2
		else
			return self._mux_single_3
		end
	else
		if channel1 == 0 and channel2 == 1 then
			return self._mux_diff_0_1
		elseif channel1 == 0 and channel2 == 3 then
			return self._mux_diff_0_3
		elseif channel1 == 1 and channel3 == 3 then
			return self._mux_diff_1_3
		elseif channel1 == 2 and channel3 == 3 then
			return self._mux_diff_2_3
		else
			print("\nDifferential between those pins not possible.")
			do return end
		end
	end
end

local read = function(self, channel1, channel2)
	config = bit.bor(self._cque_none,
		self._clat_nonlat,           
		self._cpol_actvlow,          
		self._cmode_trad,            
		self._sps_1600,            
		self._mode_single,           
		self._os_single,     -- value of config at this point is 0x8183
		self.gain,
		calc_mux(self, channel1, channel2)
	)
	
	write_register(self, 0x01, config);
	
	tmr.delay(1000) -- delay for conversion result
	
	return read_register(self)
end

local comparator_start = function(self, low_threshold, high_threshold, channel1, channel2, config)
	config = config or bit.bor(self._cque_1conv,   -- Comparator enabled and asserts on 1 match
        self._clat_latch,    -- Latching mode
        self._cpol_actvlow,  -- ALERT off when above threshold
        self._cmode_trad,    -- compare against low_threshold
        self._sps_1600,    
        self._mode_contin   -- Continuous conversion mode.
    )
	
	config = bit.bor(config, self.gain, calc_mux(self, channel1, channel2))
	
	write_register(self, 0x02, bit.lshift(low_threshold, 4))
	write_register(self, 0x03, bit.lshift(high_threshold, 4))
	write_register(self, 0x01, config)
end

local comparator_stop = function(self)
	config = bit.bor(self._cque_none,
		self._sps_1600,
		self._mode_single)
	write_register(self, 0x01, config)
end

local read_last_conversion = function(self)
	tmr.delay(1000) -- delay for conversion result
	return read_register(self)
end

-- instance metatable
local meta = {
	__index = {
		read = read,
		comparator_start = comparator_start,
		comparator_stop = comparator_stop,
		read_last_conversion = read_last_conversion,
		write_register = write_register,
		read_register = read_register,
		calc_mux = calc_mux,
	},
}

-- create new ADS1015 instance
return function(sda, scl, adr)
	local self = setmetatable({
		_adr = adr or 0x48,
		_sda = sda or 3,
		_scl = scl or 4,
		_os_mask      = 0x8000,
		_os_single    = 0x8000,  -- Write: Set to start a single-conversion
		_os_busy      = 0x0000,  -- Read: Bit = 0 when conversion is in progress
		_os_notbusy   = 0x8000,  -- Read: Bit = 1 when device is not performing a conversion
								
		_mux_mask     = 0x7000,  
		_mux_diff_0_1 = 0x0000,  -- Differential P = AIN0, N = AIN1 (default)
		_mux_diff_0_3 = 0x1000,  -- Differential P = AIN0, N = AIN3
		_mux_diff_1_3 = 0x2000,  -- Differential P = AIN1, N = AIN3
		_mux_diff_2_3 = 0x3000,  -- Differential P = AIN2, N = AIN3
		_mux_single_0 = 0x4000,  -- Single-ended AIN0
		_mux_single_1 = 0x5000,  -- Single-ended AIN1
		_mux_single_2 = 0x6000,  -- Single-ended AIN2
		_mux_single_3 = 0x7000,  -- Single-ended AIN3
								
		_pga_mask     = 0x0E00,  
		_pga_6_144v   = 0x0000,  -- +/-6.144V range = Gain 2/3 = 0
		_pga_4_096v   = 0x0200,  -- +/-4.096V range = Gain 1 = 512
		_pga_2_048v   = 0x0400,  -- +/-2.048V range = Gain 2 (default) = 1024
		_pga_1_024v   = 0x0600,  -- +/-1.024V range = Gain 4 = 1536
		_pga_0_512v   = 0x0800,  -- +/-0.512V range = Gain 8 = 2048
		_pga_0_256v   = 0x0A00,  -- +/-0.256V range = Gain 16 = 2560
								
		_mode_mask    = 0x0100,  
		_mode_contin  = 0x0000,  -- Continuous conversion mode
		_mode_single  = 0x0100,  -- Power-down single-shot mode (default)
								
		_sps_mask     = 0x00E0,  
		_sps_128      = 0x0000,  -- 128 samples per second
		_sps_250      = 0x0020,  -- 250 samples per second
		_sps_490      = 0x0040,  -- 490 samples per second
		_sps_920      = 0x0060,  -- 920 samples per second
		_sps_1600     = 0x0080,  -- 1600 samples per second (default)
		_sps_2400     = 0x00A0,  -- 2400 samples per second
		_sps_3300     = 0x00C0,  -- 3300 samples per second
								
		_cmode_mask   = 0x0010,  
		_cmode_trad   = 0x0000,  -- Traditional comparator with hysteresis (default)
		_cmode_window = 0x0010,  -- Window comparator
								
		_cpol_mask    = 0x0008,  
		_cpol_actvlow = 0x0000,  -- ALERT/RDY pin is low when active (default)
		_cpol_actvhi  = 0x0008,  -- ALERT/RDY pin is high when active
								
		_clat_mask    = 0x0004,  -- Determines if ALERT/RDY pin latches once asserted
		_clat_nonlat  = 0x0000,  -- Non-latching comparator (default)
		_clat_latch   = 0x0004,  -- Latching comparator
								
		_cque_mask    = 0x0003,  
		_cque_1conv   = 0x0000,  -- Assert ALERT/RDY after one conversions
		_cque_2conv   = 0x0001,  -- Assert ALERT/RDY after two conversions
		_cque_4conv   = 0x0002,  -- Assert ALERT/RDY after four conversions
		_cque_none    = 0x0003,  -- Disable the comparator and put ALERT/RDY in high state (default)
	}, meta)
	
	-- set GAIN values
	self.GAIN_TWOTHIRDS    = self._pga_6_144v
	self.GAIN_ONE          = self._pga_4_096v
	self.GAIN_TWO          = self._pga_2_048v
	self.GAIN_FOUR         = self._pga_1_024v
	self.GAIN_EIGHT        = self._pga_0_512v
	self.GAIN_SIXTEEN      = self._pga_0_256v
	
	self.gain = self.GAIN_TWO -- set default gain
	
	i2c.setup(0, self._sda, self._scl, i2c.SLOW)
	
	return self
end