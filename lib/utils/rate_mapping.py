"""Rate mapping"""

def sticks_to_rates(stick, max_rate_dps=600.0, expo=0.3, super_rate=0.7):
    def shape(x):
        x_e = x * (1 - expo) + x**3 * expo  # expo curve
        denom = max(1e-6, 1 - abs(x_e) * super_rate)
        return x_e / denom
    return max_rate_dps * shape(stick)

class RateMapper:
    def __init__(self, max_rates_dps=(600, 600, 500), expo=(0.25, 0.25, 0.20), 
                 super_rates=(0.7, 0.7, 0.6)):
        self.max_rates_dps = max_rates_dps
        self.expo = expo
        self.super_rates = super_rates
    
    def map_sticks_to_rates(self, roll_stick, pitch_stick, yaw_stick):
        from .data_structures import AxisRates
        
        roll_rate = sticks_to_rates(roll_stick, self.max_rates_dps[0], 
                                   self.expo[0], self.super_rates[0])
        pitch_rate = sticks_to_rates(pitch_stick, self.max_rates_dps[1],
                                    self.expo[1], self.super_rates[1]) 
        yaw_rate = sticks_to_rates(yaw_stick, self.max_rates_dps[2],
                                  self.expo[2], self.super_rates[2])
        
        return AxisRates(p=roll_rate, q=pitch_rate, r=yaw_rate)