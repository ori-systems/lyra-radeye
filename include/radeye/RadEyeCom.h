#ifndef RAD_EYE_COM_H
#define RAD_EYE_COM_H

#include <iostream>
#include <string>
#include <sys/ioctl.h> // input output control 
#include <fcntl.h>     // File Control Definitions          
#include <termios.h>   // POSIX Terminal Control Definitions
#include <unistd.h>    // UNIX Standard Definitions         
#include <errno.h>     // ERROR Number Definitions  
   
#include <algorithm>
#include <sstream>
#include <algorithm>
#include <iterator>

class RadEyeCom
{
	bool portOpened = false;
	int fd; //file descriptor
	struct termios SerialPortSettings; // structure for the serial port settings
	std::string file_name = "";
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
		
		for(std::size_t i = 0;i<10;i++) //try recieving 10 times
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
	
	std::string SendCommand(std::string com)
	{
		usleep(5000);//wait 5ms after SendWake()
		com = ">"+com;//start byte
		if (com.length() > 1)
		{
			std::string ret = "";
			char command[10];
			strcpy(command, com.c_str());

			command[com.length()]=0x0A;//end byte
			char read_buffer[32];
			write(fd, &command, com.length()+1);//wite command + end byte
						
			usleep(600);//wait for a response
			
			for(std::size_t i = 0;i<400;i++) //try recieving 100 times
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


	std::string ReadSerial()
	{
		std::string ret = "";
		char read_buffer[32];		
		for(std::size_t i = 0;i<400;i++) //try recieving 100 times
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

	void init(std::string file_name_)
	{
		file_name = file_name_;
		fd = open(file_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); //open file for read write
		{// file return checks (did not open | not tty | serial settings failed)
				
			if (fd==-1)
			{
				#ifndef RAD_EYE_NO_DEBUG
				std::cerr << "Error Cannot Open File " << file_name << std::endl;
				#endif
				return;
			}
			else
			{
				#ifndef RAD_EYE_NO_DEBUG
				std::cout << "Successfully opened file " << file_name << std::endl;
				#endif
			}
			if(!isatty(fd))
			{
				#ifndef RAD_EYE_NO_DEBUG
				std::cerr << file_name << " is not a tty" << std::endl;
				#endif
				return;
			}
			if(tcgetattr(fd, &SerialPortSettings) < 0)
			{
				#ifdef RAD_EYE_DEBUG
				std::cout << "could not load serial port settings for " << file_name << std::endl;
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
				#ifndef RAD_EYE_NO_DEBUG
				std::cerr << "Error could not set baude rate for " << file_name << std::endl;
				#endif
			}
			
			if(tcsetattr(fd, TCSAFLUSH, &SerialPortSettings) < 0)
			{
				#ifndef RAD_EYE_NO_DEBUG
				std::cout << "Error could not set serial port settings for " << file_name << std::endl;
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
		#ifndef RAD_EYE_NO_DEBUG
		std::cout << "closed file " << file_name << std::endl;
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
	
	std::vector <float> DataSetToFloats(std::string DataSet)
	{
		std::istringstream iss(DataSet);
		std::vector<std::string> tokens{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
		std::vector<float> data;
		data.resize(tokens.size());
		for(std::size_t i = 0; i<tokens.size();i++)
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
	std::vector <T> DataSetToVector(std::string DataSet)
	{
		std::istringstream iss(DataSet);
		std::vector<std::string> tokens{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
		std::vector<T> data;
		data.resize(tokens.size());
		for(std::size_t i = 0; i<tokens.size();i++)
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
	std::string CommandString(std::string command)
	{
		std::string ret = "";
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
		std::string cycleTime = CommandString("TR");
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
		std::string Command = "TW" + std::to_string(time);
		CommandString(Command);
	}
	void InitializeHistoryReadout()
	{
		CommandString("HI");
	}
	virtual std::vector<int> ReadNextHistoryDataSet()
	{
		return DataSetToVector<int>(CommandString("+"));
	}
	virtual std::vector<int> ReadLastHistoryDataSet()
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
	virtual std::vector<int> ReadNextEventLog()
	{
		return DataSetToVector<int>(CommandString("E+"));
	}
	virtual std::vector<int> ReadLastEventLog()
	{
		return DataSetToVector<int>(CommandString("E-"));
	}
	void ClearEventLog()
	{
		CommandString("EC");
	}
	
	
	//4.3 Date and Time
	
	std::string GetDateTime()
	{
		int Idate[]={0,0,0,0,0,0};//{Y,M,D,H,M,S}
		std::string Date = CommandString("ZR");
		if (Date.length() == 13)
		{
			for(std::size_t i=0;i<6;i++)
				Idate[i]=(Date[i*2]-'0')*10+(Date[i*2+1]-'0');
			Idate[0] += 2000;
		}
		Date = std::to_string(Idate[2]) + "/" + std::to_string(Idate[1]) + "/" + std::to_string(Idate[0]) + " " + std::to_string(Idate[3]) + ":" + std::to_string(Idate[4]) + ":" + std::to_string(Idate[5]);
		return Date;
	}
	void SetDateTime(unsigned int year,unsigned int month,unsigned int day,unsigned int hours,unsigned int minutes,unsigned int seconds)
	{
		year %= 2000;
		if(year <= 99 && month <=12 && day <=31 && hours <24 && minutes < 60 && seconds < 60)
		{
			std::string CommandDateTime = std::to_string(year)+std::to_string(month)+std::to_string(day)+std::to_string(hours)+std::to_string(minutes)+std::to_string(seconds);
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
		std::cout << "this is the virtual function 'ReadMenuConfiguration()' needs implementing for specific RadEye" << std::endl;
		std::cout << CommandString("mR") << std::endl;
	}
	virtual void WriteMenuConfiguration(int configuration)
	{
		std::cout << "this is the virtual function 'WriteMenuConfiguration()' needs implementing for specific RadEye"  << std::endl;
		//CommandString("mWHex");
	}
	virtual void ReadConfigurationflag1()
	{
		std::cout << "this is the virtual function 'Readconfigurationflag1()' needs implementing for specific RadEye"  << std::endl;
		std::cout << CommandString("fR") << std::endl;
	}
	virtual void WriteConfigurationFlag1()
	{
		std::cout << "this is the virtual function 'WriteConfigurationFlag1()' needs implementing for specific RadEye"  << std::endl;
		//CommandString("fWHex");
	} 
	virtual void ReadConfigurationflag2()
	{
		std::cout << "this is the virtual function 'Readconfigurationflag1()' needs implementing for specific RadEye"  << std::endl;
		std::cout << CommandString("kR") << std::endl;
	}
	virtual void WriteConfigurationFlag2()
	{
		std::cout << "this is the virtual function 'WriteConfigurationFlag1()' needs implementing for specific RadEye"  << std::endl;
		//CommandString("kWHex");
	} 
	virtual void ReadConfigurationflag3()
	{
		std::cout << "this is the virtual function 'Readconfigurationflag1()' needs implementing for specific RadEye"  << std::endl;
		std::cout << CommandString("KR") << std::endl;
	}
	virtual void WriteConfigurationFlag3()
	{
		std::cout << "this is the virtual function 'WriteConfigurationFlag1()' needs implementing for specific RadEye"  << std::endl;
		//CommandString("KWHex");
	} 
	std::string ReadMenuLanguage()
	{
		std::string Language = CommandString("sR");
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
	void SetMenuLanguage(std::string Lang)
	{
		if(Lang[0] == 'E' ||Lang[0] == 'e' ||Lang[0] == '0') //english
			CommandString("sW0");
		else if(Lang[0] == 'G' ||Lang[0] == 'g' ||Lang[0] == '1')//german
			CommandString("sW1");
		else if(Lang[0] == 'F' ||Lang[0] == 'f' ||Lang[0] == '2')//french
			CommandString("sW2");
		else
			std::cout << "unknown language" << std::endl;
	}
	int ReadTimeoutOfAlarmLatching()
	{
		std::string Time = CommandString("ART");
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
			std::string command = "AWT" + std::to_string(time);
			CommandString(command);
		}
	}
	
	//4.6 serial interface
	
	int ReadSerialTimeout()
	{
		std::string Time = CommandString("ARS");
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
			std::string command = "AWS" + std::to_string(time);
			CommandString(command);
	}
	int ReadingResetTransferErrorCounts()
	{
		std::string Errors = CommandString("t");
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
	
	std::string ReadCalibrationDate()
	{
		int Idate[]={0,0,0};//{Y,M,D}
		std::string Date = CommandString("WR");
		if (Date.length() == 7)
		{
			for(std::size_t i=0;i<3;i++)
				Idate[i]=(Date[i*2]-'0')*10+(Date[i*2+1]-'0');
			Idate[0] += 2000;
		}
		Date = std::to_string(Idate[2]) + "/" + std::to_string(Idate[1]) + "/" + std::to_string(Idate[0]);
		return Date;
	}
	virtual void ReadCalibrationFactor() // lol
	{
		std::cout << "this is the virtual function 'ReadCalibrationFactor()' needs implementing for specific RadEye"  << std::endl;
		std::cout << CommandString("$") << std::endl;
	}
	int ReadDeviceSerialNumber()
	{
		std::string SerialNumber = CommandString("#R");
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
	
	std::string RadeEyeType()
	{
		std::string Type = CommandString("Vx");
		std::istringstream iss(Type);
		
		std::vector<std::string> tokens{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
		
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
	
	std::string DeviceDescriptionReadingText()
	{
		return CommandString("DR");
	}
	void WriteText(std::string text)
	{
		int numberOfSends = text.length()/20 + 1;
		if (numberOfSends > 10) numberOfSends = 10;
		for(int i=0; i<numberOfSends;i++)
		{
			std::string command = "DW" + std::to_string(i) + text.substr(i*20,20);
			//cout << command << std::endl;
			CommandString(command);
			sleep(1);
		}
	}
	std::string ReadTextLine(unsigned int line)
	{
		if (line < 4)
		{
			std::string command = "dR" + std::to_string(line);
			return CommandString(command);
		}
		else
		return "";
	}
	void WriteTextLine(unsigned int line, std::string text)
	{
		if (line < 4)
		{
			std::string command = "dW" + std::to_string(line) + text.substr(0,16);
			CommandString(command);
		}
	}
	
	// 4.11 Measurement Values
	virtual std::vector<int> ReadRawCountRates()
	{
		return DataSetToVector<int>(CommandString("Z"));
	}
	virtual std::vector<int> ReadFilteredCountRates()
	{
		return DataSetToVector<int>(CommandString("z"));
	}
	virtual std::vector<double> DoseRate()
	{
		return DataSetToVector<double>(CommandString("R"));
	}
	virtual std::vector<int> AccumulatedDose()
	{
		return DataSetToVector<int>(CommandString("D"));
	}
	void ClearAccumulatedDose()
	{
		std::cout << CommandString("clr") << std::endl;
	}
	float ReadBatteryVoltage()
	{
		std::string Voltage = CommandString("Ux");
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
		std::cout << "this is the virtual function 'ReadStatus()' needs implementing for specific RadEye"  << std::endl;
		std::cout << CommandString("F") << std::endl;
	}
	float ReadTemperature()
	{
		std::cout << "reading temperature" << std::endl;
		std::string Temperature = CommandString("tR");
		std::cout << "Temperature = CommandString(tR) = " << Temperature << std::endl;
		float ITemperature = 0;
		try
		{
			ITemperature = stof(Temperature);
		}
		catch(...)
		{
			ITemperature = -1;
		}
		std::cout << "temperature is" << ITemperature << std::endl;
		return ITemperature;
	}
	
	virtual std::vector<int> ReadNominalValueHighVoltage()
	{
		return DataSetToVector<int>(CommandString("HR"));
	}
	
};
#endif //RAD_EYE_COM_H
