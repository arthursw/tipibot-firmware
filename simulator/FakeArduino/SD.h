/*

SD - a slightly more friendly wrapper for sdfatlib

This library aims to expose a subset of SD card functionality
in the form of a higher level "wrapper" object.

License: GNU General Public License V3
(Because sdfatlib is licensed with this.)

(C) Copyright 2010 SparkFun Electronics

*/

#ifndef __SD_H__
#define __SD_H__

#include <iostream>
#include <fstream>
#include <string>

#include "FakeArduino.h"


#define FILE_READ 0
#define FILE_WRITE 1

namespace SDLib {

	class File {
	private:
		char _name[13]; // our name

	public:
		std::fstream fs;
		File(){}
		File(std::string filename, uint8_t mode) {
			this->fs = std::fstream(filename, mode == FILE_READ ? std::fstream::in : mode == FILE_WRITE ? std::fstream::out : std::fstream::in | std::fstream::out | std::fstream::app);
			if (!this->fs.is_open()) {
				std::cout << "failed to open " << filename << '\n';
			}
		};

		size_t write(uint8_t c) {
			fs << c;
			return 1;
		}

		size_t write(const uint8_t *buf, size_t size) {
			std::string str((char *)buf, size);
			fs << str;
			return 1;
		}
		int read(){
			char c;
			fs.readsome(&c, 1);
			return c;
		}
		int peek(){
			return fs.peek();
		}
		int available() {
			return fs.eof();
		};
		void flush();
		int read(char *buf, uint16_t nbyte) {
			return fs.readsome(buf, nbyte);
		};
		boolean seek(uint32_t pos){
			fs.seekg(pos);
			return true;
		};
		uint32_t position();
		uint32_t size();
		void close() {
			fs.close();
		}
		operator bool() {
			return true;
		}
		char * name();

		boolean isDirectory(void);
		File openNextFile(uint8_t mode = 0);
		void rewindDirectory(void);


	};

	class SDClass {

	public:

		SDClass() {

		}
		// This needs to be called to set up the connection to the SD card
		// before other methods are used.
		boolean begin(uint8_t csPin = 0){return true;}
		boolean begin(uint32_t clock, uint8_t csPin){return true;}

		// Open the specified file/directory with the supplied mode (e.g. read or
		// write, etc). Returns a File object for interacting with the file.
		// Note that currently only one file can be open at a time.
		File open(const char *filename, uint8_t mode = FILE_READ){
			return File(filename, mode);
		}
		File open(const string &filename, uint8_t mode = FILE_READ) { return open( filename.c_str(), mode ); }

		// Methods to determine if the requested file path exists.
		boolean exists(const char *filepath);
		boolean exists(const string &filepath) { return exists(filepath.c_str()); }

		// Create the requested directory heirarchy--if intermediate directories
		// do not exist they will be created.
		boolean mkdir(const char *filepath);
		boolean mkdir(const string &filepath) { return mkdir(filepath.c_str()); }

		// Delete the file.
		boolean remove(const char *filepath);
		boolean remove(const string &filepath) { return remove(filepath.c_str()); }

		boolean rmdir(const char *filepath);
		boolean rmdir(const string &filepath) { return rmdir(filepath.c_str()); }

	private:

		// This is used to determine the mode used to open a file
		// it's here because it's the easiest place to pass the
		// information through the directory walking function. But
		// it's probably not the best place for it.
		// It shouldn't be set directly--it is set via the parameters to `open`.
		int fileOpenMode;

		friend class File;

	};


};


// We enclose File and SD classes in namespace SDLib to avoid conflicts
// with others legacy libraries that redefines File class.

// This ensure compatibility with sketches that uses only SD library
using namespace SDLib;

extern SDClass SD;

// This allows sketches to use SDLib::File with other libraries (in the
// sketch you must use SDFile instead of File to disambiguate)


#endif
