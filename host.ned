//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//
//
//--------FINAL_PROJECT----------
//  Saar Ben-Yochana - 313234155
//  Efi Korenfeld - 208838359
//-------------------------------

// Creating a host module:
simple Host
{
    parameters:
        int R = default(10);							// Reward
        int ttl = default(6);	
        int net = default(1);
        int TARGET = default(5);
        bool OPT = default(false);						// To indicate if we use OPT algorith, otherwise- TSOR
        string source_path = default("Network1.host");  // Source path, to send statistics signals during simulation
        volatile double sendingInterval @unit(s); 
        
        // Statistic collection signals:
		@signal[costs](type="int");
        @signal[rewards](type="int");
        
        @display("i=device/wifilaptop");
    gates:
        inout interface[];								// Connection between hosts
        input control @loose;							// Statistic collection
}
