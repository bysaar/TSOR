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
//--------FINAL_PROJECT----------
//  Saar Ben-Yochana - 313234155
//  Efi Korenfeld - 208838359
//-------------------------------

// Packet with the required fields for running the algorithm:
message pkt extends cPacket
{
	int src;
	int dst;
	int ttl;
	int type;
	int sequence_number;
	int hop_count;
	int lack_type;
	double Vn_tag;
}

