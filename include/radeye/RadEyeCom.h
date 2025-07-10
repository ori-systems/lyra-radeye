#ifndef RAD_EYE_COM_H
#define RAD_EYE_COM_H

#include <iostream>
#include <string.h>
#include <sys/ioctl.h> // input output control 
#include <fcntl.h>     // File Control Definitions          
#include <termios.h>   // POSIX Terminal Control Definitions
#include <unistd.h>    // UNIX Standard Definitions         
#include <errno.h>     // ERROR Number Definitions  
   
#include <algorithm>
#include <sstream>
#include <algorithm>
#include <iterator>
using namespace std;

class RadEyeCom
{
	bool portOpened = false;
	int fd; //file descriptor
	struct termios SerialPortSettings; // structure for the serial port settings
	string file_name = "";
	bool printout = false;
	int time_delay_ms = 0;
	
	////////////////////////////////////////////////////////////////////
	//                                                                //
	//            Numbers in comments refer to sections               //
	//              in the Rad eye protocol document                  //
	//                                                                //
	////////////////////////////////////////////////////////////////////
	
	
	
	////////////////////////////////////////////////////////////////////
	//                                                                //
	//                       RadEye protocol 2.0                      //
	//                                                                //
	////////////////////////////////////////////////////////////////////
	
	bool SendWake()// start of communication
	{
		char start_command[]="@";
		char read_buffer[32];
		write(fd, &start_command, 1);
					
		usleep(600);//wait for a response
		
		for(int i = 0;i<10;i++) //try recieving 10 times
		{			
			if(read(fd,&read_buffer,1)>0)
			{
				//cout << read_buffer;
				if(read_buffer[0] == '>')
				{
					return true;
				}
			}
			usleep(50);
		}
		return false;
	};
	
	string SendCommand(string com)
	{
		usleep(5000);//wait 5ms after SendWake()
		com = ">"+com;//start byte
		if (com.length() > 1)
		{
			string ret = "";
			char command[10];
			strcpy(command, com.c_str());

			command[com.length()]=0x0A;//end byte
			char read_buffer[32];
			write(fd, &command, com.length()+1);//wite command + end byte
						
			usleep(600);//wait for a response
			
			for(int i = 0;i<400;i++) //try recieving 100 times
			{			
				if(int bytes=read(fd,&read_buffer,1)>0)
				{
					i=0;
					for(int j=0;j<bytes;j++)
						ret += read_buffer[j];
				}
				usleep(50);
			}
			
			ret.erase(std::remove(ret.begin(), ret.end(), '>'), ret.end());
			ret.erase(std::remove(ret.begin(), ret.end(), '#'), ret.end());
			ret.erase(std::remove(ret.begin(), ret.end(),'\n'), ret.end());
			return ret;
		}
		else
		{
			return "nope";
		}
	};
		



	public:


	string ReadSerial()
	{
		string ret = "";
		char read_buffer[32];		
		for(int i = 0;i<400;i++) //try recieving 100 times
		{			
			if(int bytes=read(fd,&read_buffer,1)>0)
			{
				i=0;
				for(int j=0;j<bytes;j++)
					ret += read_buffer[j];
			}
			usleep(50);
		}

		ret.erase(std::remove(ret.begin(), ret.end(), '>'), ret.end());
		ret.erase(std::remove(ret.begin(), ret.end(), '#'), ret.end());
		ret.erase(std::remove(ret.begin(), ret.end(),'\n'), ret.end());


		return ret;

	};


	
	////////////////////////////////////////////////////////////////////
	//                                                                //
	//                 Comms Initialise Uninitialise 2.0              //
	//                                                                //
	////////////////////////////////////////////////////////////////////
	RadEyeCom()
	{}

	void init(string file_name_)
	{
		file_name = file_name_;
		fd = open(file_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); //open file for read write
		{// file return checks (did not open | not tty | serial settings failed)
				
			if (fd==-1)
			{
				#ifdef RAD_EYE_DEBUG
				cout << "Error Cannot Open File " << file_name << endl;
				#endif
				return;
			}
			else
			{
				#ifdef RAD_EYE_DEBUG
				cout << "Successfully opened file " << file_name << endl;
				#endif
			}
			if(!isatty(fd))
			{
				#ifdef RAD_EYE_DEBUG
				cout << file_name << " is not a tty" << endl;
				#endif
				return;
			}
			if(tcgetattr(fd, &SerialPortSettings)<0)
			{
				#ifdef RAD_EYE_DEBUG
				cout << "could not load serial port settings for " << file_name << endl;
				#endif
				return;
			}
		}
		{//configurations (input, output, local and control flags; and control chars)
			
			//input flags
			SerialPortSettings.c_iflag &= ~ IGNBRK; // Ignore BREAK condition on input
			SerialPortSettings.c_iflag &= ~ BRKINT; // a BREAK reads as a null byte ('\0')
			SerialPortSettings.c_iflag &= ~ ICRNL;  // Don't translate carriage return to newline on input
			SerialPortSettings.c_iflag &= ~ INLCR;  // Don't Translate NL to CR on input
			SerialPortSettings.c_iflag &= ~ PARMRK; // Don't mark parity errors
			SerialPortSettings.c_iflag &= ~ INPCK;  // Disable input parity check
			SerialPortSettings.c_iflag &= ~ ISTRIP; // Don't strip 8th bit
			SerialPortSettings.c_iflag &= ~ IXON;   // Disable XON/XOFF flow control on output

			//output flags
			SerialPortSettings.c_oflag = 0;
			
			//local flags
			SerialPortSettings.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

			//control flags
			SerialPortSettings.c_cflag &= ~CSIZE; 
			SerialPortSettings.c_cflag |= PARENB; // even parity
			SerialPortSettings.c_cflag |= CSTOPB; // two stop bits
			
			SerialPortSettings.c_cflag &= ~CS8;    // seven bits
			SerialPortSettings.c_cflag |= CS7;    // seven bits
			
			
			//control chars
			SerialPortSettings.c_cc[VMIN]  = 1;
			SerialPortSettings.c_cc[VTIME] = 0;
		
			//baude rate
			if(cfsetispeed(&SerialPortSettings, B9600) < 0 || cfsetospeed(&SerialPortSettings, B9600) < 0) 
			{
				#ifdef RAD_EYE_DEBUG
				cout << "Error could not set baude rate for " << file_name << endl;
				#endif
			}
			
			if(tcsetattr(fd, TCSAFLUSH, &SerialPortSettings) < 0)
			{
				#ifdef RAD_EYE_DEBUG
				cout << "Error could not set serial port settings for " << file_name << endl;
				#endif
			}
			
		}
		{//RTS DTR stuff (Dont know if this does anything)
			int RTS_flag,DTR_flag;
			RTS_flag = TIOCM_RTS;
			DTR_flag = TIOCM_DTR;
			ioctl(fd,TIOCMBIS,&RTS_flag);//Set   RTS pin
			ioctl(fd,TIOCMBIS,&DTR_flag);//Set   DTR pin
		}
		portOpened = true;
		sleep(1);
	};
	
	virtual ~RadEyeCom()
	{
		close(fd);
		#ifdef RAD_EYE_DEBUG
		cout << "closed file " << file_name << endl;
		#endif
	};

	////////////////////////////////////////////////////////////////////
	//                                                                //
	//                      Class Specific Interface                  //
	//                                                                //
	////////////////////////////////////////////////////////////////////

	bool IsPortOpen()
	{
		return portOpened;
	}

	void SetTimeDelay(int delay_ms)
	{
		time_delay_ms = delay_ms;
	}
	
	vector <float> DataSetToFloats(string DataSet)
	{
		istringstream iss(DataSet);
		vector<string> tokens{istream_iterator<string>{iss}, istream_iterator<string>{}};
		vector<float> data;
		data.resize(tokens.size());
		for(int i = 0; i<tokens.size();i++)
		{
			try
			{
				data[i] = stof(tokens[i]);
			}
			catch(...)
			{
				data[i] = -1;
			}
		}
		return data;
	}
	
	template<typename T>
	vector <T> DataSetToVector(string DataSet)
	{
		istringstream iss(DataSet);
		vector<string> tokens{istream_iterator<string>{iss}, istream_iterator<string>{}};
		vector<T> data;
		data.resize(tokens.size());
		for(int i = 0; i<tokens.size();i++)
		{
			try
			{
				if(std::is_integral<T>::value)data[i] = stoi(tokens[i]);
				if(std::is_floating_point<T>::value)data[i] = stof(tokens[i]);
			}
			catch(...)
			{
				data[i] = -1;
			}
		}
		return data;
	}
	
	
	
    ////////////////////////////////////////////////////////////////////
	//                                                                //
	//                         User Interface 4.0                     //
	//                                                                //
	////////////////////////////////////////////////////////////////////	
	
	
	// raw command interface, pass a string to give to RadEye it returns a string
	string CommandString(string command)
	{
		string ret = "";
		for(int i = 0; i < 3; i++) //try 3 times 
		{
			usleep(10000);// minimum wait between sends (magic number)
			for(int j = 0; j < time_delay_ms; j++)
				usleep(1000); //user defined delay time
			if(SendWake())// send the '@' to the RadEye to initialise comms 
			{
				ret = SendCommand(command);
				if(ret != "")
					return ret;
			}
		}
		return "";
	}
	//4.1 History
	int ReadHistoryCycleTime()
	{
		string cycleTime = CommandString("TR");
		int IcycleTime;
		try
		{
			IcycleTime = stoi(cycleTime);
		}
		catch(...)
		{
			IcycleTime = 0;
		}
		return IcycleTime;
	}
	void SetHistoryCycleTime(int time)
	{
		string Command = "TW" + to_string(time);
		CommandString(Command);
	}
	void InitializeHistoryReadout()
	{
		CommandString("HI");
	}
	virtual vector<int> ReadNextHistoryDataSet()
	{
		return DataSetToVector<int>(CommandString("+"));
	}
	virtual vector<int> ReadLastHistoryDataSet()
	{
		return DataSetToVector<int>(CommandString("-"));
	}
	void ClearHistory()
	{
		CommandString("ph");
	}
	
	//4.2 Event Log
	
	void InitializeReadoutEventLog()
	{
		CommandString("EI");
	}
	virtual vector<int> ReadNextEventLog()
	{
		return DataSetToVector<int>(CommandString("E+"));
	}
	virtual vector<int> ReadLastEventLog()
	{
		return DataSetToVector<int>(CommandString("E-"));
	}
	void ClearEventLog()
	{
		CommandString("EC");
	}
	
	
	//4.3 Date and Time
	
	string GetDateTime()
	{
		int Idate[]={0,0,0,0,0,0};//{Y,M,D,H,M,S}
		string Date = CommandString("ZR");
		if (Date.length() == 13)
		{
			for(int i=0;i<6;i++)
				Idate[i]=(Date[i*2]-'0')*10+(Date[i*2+1]-'0');
			Idate[0] += 2000;
		}
		Date = to_string(Idate[2]) + "/" + to_string(Idate[1]) + "/" + to_string(Idate[0]) + " " + to_string(Idate[3]) + ":" + to_string(Idate[4]) + ":" + to_string(Idate[5]);
		return Date;
	}
	void SetDateTime(unsigned int year,unsigned int month,unsigned int day,unsigned int hours,unsigned int minutes,unsigned int seconds)
	{
		year %= 2000;
		if(year <= 99 && month <=12 && day <=31 && hours <24 && minutes < 60 && seconds < 60)
		{
			string CommandDateTime = to_string(year)+to_string(month)+to_string(day)+to_string(hours)+to_string(minutes)+to_string(seconds);
		}
	}
	
	//4.4 EEPROM
	
	void StoreConfigurationToEEPROM()
	{
		CommandString("EW");
	}
	
	//4.5 Configuration
	
	virtual void ReadMenuConfiguration()
	{
		cout << "this is the virtual function 'ReadMenuConfiguration()' needs implementing for specific RadEye" << endl;
		cout << CommandString("mR") << endl;
	}
	virtual void WriteMenuConfiguration(int configuration)
	{
		cout << "this is the virtual function 'WriteMenuConfiguration()' needs implementing for specific RadEye"  << endl;
		//CommandString("mWHex");
	}
	virtual void ReadConfigurationflag1()
	{
		cout << "this is the virtual function 'Readconfigurationflag1()' needs implementing for specific RadEye"  << endl;
		cout << CommandString("fR") << endl;
	}
	virtual void WriteConfigurationFlag1()
	{
		cout << "this is the virtual function 'WriteConfigurationFlag1()' needs implementing for specific RadEye"  << endl;
		//CommandString("fWHex");
	} 
	virtual void ReadConfigurationflag2()
	{
		cout << "this is the virtual function 'Readconfigurationflag1()' needs implementing for specific RadEye"  << endl;
		cout << CommandString("kR") << endl;
	}
	virtual void WriteConfigurationFlag2()
	{
		cout << "this is the virtual function 'WriteConfigurationFlag1()' needs implementing for specific RadEye"  << endl;
		//CommandString("kWHex");
	} 
	virtual void ReadConfigurationflag3()
	{
		cout << "this is the virtual function 'Readconfigurationflag1()' needs implementing for specific RadEye"  << endl;
		cout << CommandString("KR") << endl;
	}
	virtual void WriteConfigurationFlag3()
	{
		cout << "this is the virtual function 'WriteConfigurationFlag1()' needs implementing for specific RadEye"  << endl;
		//CommandString("KWHex");
	} 
	string ReadMenuLanguage()
	{
		string Language = CommandString("sR");
		int ILanguage = 0;
		try
		{
			ILanguage = stof(Language);
		}
		catch(...)
		{
			ILanguage = -1;
		}
		switch (ILanguage)
		{
			case 0:
			Language = "English";
			break;
			case 1:
			Language = "German";
			break;
			case 2:
			Language = "French";
			break;
			default:
			Language = "Failed to Read";
		}
		return Language;
	}
	void SetMenuLanguage(string Lang)
	{
		if(Lang[0] == 'E' ||Lang[0] == 'e' ||Lang[0] == '0') //english
			CommandString("sW0");
		else if(Lang[0] == 'G' ||Lang[0] == 'g' ||Lang[0] == '1')//german
			CommandString("sW1");
		else if(Lang[0] == 'F' ||Lang[0] == 'f' ||Lang[0] == '2')//french
			CommandString("sW2");
		else
			cout << "unknown language" << endl;
	}
	int ReadTimeoutOfAlarmLatching()
	{
		string Time = CommandString("ART");
		int ITime = 0;
		try
		{
			ITime = stof(Time);
		}
		catch(...)
		{
			ITime= -1;
		}
		return ITime;
	}
	void SetTimeoutOfAlarmLatching(unsigned int time)
	{
		if (time < 256)
		{
			string command = "AWT" + to_string(time);
			CommandString(command);
		}
	}
	
	//4.6 serial interface
	
	int ReadSerialTimeout()
	{
		string Time = CommandString("ARS");
		int ITime = 0;
		try
		{
			ITime = stof(Time);
		}
		catch(...)
		{
			ITime= -1;
		}
		return ITime;
	}
	void SetSerialTimeout(unsigned int time)
	{
			string command = "AWS" + to_string(time);
			CommandString(command);
	}
	int ReadingResetTransferErrorCounts()
	{
		string Errors = CommandString("t");
		int IErrors = 0;
		try
		{
			IErrors = stof(Errors);
		}
		catch(...)
		{
			IErrors= -1;
		}
		return IErrors;
	}
	
	//4.7 Calibration
	
	string ReadCalibrationDate()
	{
		int Idate[]={0,0,0};//{Y,M,D}
		string Date = CommandString("WR");
		if (Date.length() == 7)
		{
			for(int i=0;i<3;i++)
				Idate[i]=(Date[i*2]-'0')*10+(Date[i*2+1]-'0');
			Idate[0] += 2000;
		}
		Date = to_string(Idate[2]) + "/" + to_string(Idate[1]) + "/" + to_string(Idate[0]);
		return Date;
	}
	virtual void ReadCalibrationFactor() // lol
	{
		cout << "this is the virtual function 'ReadCalibrationFactor()' needs implementing for specific RadEye"  << endl;
		cout << CommandString("$") << endl;
	}
	int ReadDeviceSerialNumber()
	{
		string SerialNumber = CommandString("#R");
		int ISerialNumber = 0;
		try
		{
			ISerialNumber = stof(SerialNumber);
		}
		catch(...)
		{
			ISerialNumber = -1;
		}

		//std::cout << ISerialNumber << std::endl;
		return ISerialNumber;
	}
	
	// 4.8 RadEye Type
	
	string RadeEyeType()
	{
		string Type = CommandString("Vx");
		istringstream iss(Type);
		
		vector<string> tokens{istream_iterator<string>{iss}, istream_iterator<string>{}};
		
		return tokens[1];
	}
	
	// 4.9 Automatic Sending
	
	void DeactivateCyclicSending()
	{
		CommandString("X0");
	}
	void ActivateCyclicSending()
	{
		CommandString("X1");
	}
	
	// 4.10 Device Description
	
	string DeviceDescriptionReadingText()
	{
		return CommandString("DR");
	}
	void WriteText(string text)
	{
		int numberOfSends = text.length()/20 + 1;
		if (numberOfSends > 10) numberOfSends = 10;
		for(int i=0; i<numberOfSends;i++)
		{
			string command = "DW" + to_string(i) + text.substr(i*20,20);
			//cout << command << endl;
			CommandString(command);
			sleep(1);
		}
	}
	string ReadTextLine(unsigned int line)
	{
		if (line < 4)
		{
			string command = "dR" + to_string(line);
			return CommandString(command);
		}
		else
		return "";
	}
	void WriteTextLine(unsigned int line, string text)
	{
		if (line < 4)
		{
			string command = "dW" + to_string(line) + text.substr(0,16);
			CommandString(command);
		}
	}
	
	// 4.11 Measurement Values
	virtual vector<int> ReadRawCountRates()
	{
		return DataSetToVector<int>(CommandString("Z"));
	}
	virtual vector<int> ReadFilteredCountRates()
	{
		return DataSetToVector<int>(CommandString("z"));
	}
	virtual vector<double> DoseRate()
	{
		return DataSetToVector<double>(CommandString("R"));
	}
	virtual vector<int> AccumulatedDose()
	{
		return DataSetToVector<int>(CommandString("D"));
	}
	void ClearAccumulatedDose()
	{
		cout << CommandString("clr") << endl;
	}
	float ReadBatteryVoltage()
	{
		string Voltage = CommandString("Ux");
		float FVoltage = 0;
		try
		{
			FVoltage = stof(Voltage)*0.1;
		}
		catch(...)
		{
			FVoltage= -1;
		}
		return FVoltage;
	}
	virtual void ReadStatus()
	{
		cout << "this is the virtual function 'ReadStatus()' needs implementing for specific RadEye"  << endl;
		cout << CommandString("F") << endl;
	}
	float ReadTemperature()
	{
		cout << "reading temperature" << endl;
		string Temperature = CommandString("tR");
		cout << "Temperature = CommandString(tR) = " << Temperature << endl;
		float ITemperature = 0;
		try
		{
			ITemperature = stof(Temperature);
		}
		catch(...)
		{
			ITemperature = -1;
		}
		cout << "temperature is" << ITemperature << endl; 
		return ITemperature;
	}
	
	virtual vector<int> ReadNominalValueHighVoltage()
	{
		return DataSetToVector<int>(CommandString("HR"));
	}
	
};
#endif //RAD_EYE_COM_H
