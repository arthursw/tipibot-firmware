##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=simulator
ConfigurationName      :=Debug
WorkspacePath          :=/Users/Arthur/Documents/PlatformIO/Projects/Tipibot/simulator
ProjectPath            :=/Users/Arthur/Documents/PlatformIO/Projects/Tipibot/simulator
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=Arthur Masson
Date                   :=10/10/2018
CodeLitePath           :="/Users/Arthur/Library/Application Support/codelite"
LinkerName             :=clang++
SharedObjectLinkerName :=clang++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=
PreprocessSuffix       :=.o.i
DebugSwitch            :=-gstab
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/$(ProjectName)
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E 
ObjectsFileList        :="simulator.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). 
IncludePCH             := 
RcIncludePath          := 
Libs                   := 
ArLibs                 :=  
LibPath                := $(LibraryPathSwitch). 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := ar rcus
CXX      := clang++
CC       := clang
CXXFLAGS :=  -g -O0 -std=c++11 $(Preprocessors)
CFLAGS   :=  -g -O0 -std=c++11 $(Preprocessors)
ASFLAGS  := 
AS       := llvm-as


##
## User defined environment variables
##
CodeLiteDir:=/private/var/folders/sz/f97g80v149b689p1jvnwybgr0000gn/T/AppTranslocation/D2C18F91-D50A-44B6-9AB4-AF2DF7C27BBA/d/codelite.app/Contents/SharedSupport/
Objects0=$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IntermediateDirectory)/FakeArduino_FakeArduino.cpp$(ObjectSuffix) $(IntermediateDirectory)/FakeArduino_SD.cpp$(ObjectSuffix) $(IntermediateDirectory)/FakeArduino_fake_serial.cpp$(ObjectSuffix) $(IntermediateDirectory)/FakeArduino_AccelStepper.cpp$(ObjectSuffix) $(IntermediateDirectory)/easywsclient.cpp$(ObjectSuffix) 



Objects=$(Objects0) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild MakeIntermediateDirs
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

MakeIntermediateDirs:
	@test -d ./Debug || $(MakeDirCommand) ./Debug


$(IntermediateDirectory)/.d:
	@test -d ./Debug || $(MakeDirCommand) ./Debug

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/main.cpp$(ObjectSuffix): main.cpp 
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/Arthur/Documents/PlatformIO/Projects/Tipibot/simulator/main.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main.cpp$(PreprocessSuffix): main.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main.cpp$(PreprocessSuffix) main.cpp

$(IntermediateDirectory)/FakeArduino_FakeArduino.cpp$(ObjectSuffix): FakeArduino/FakeArduino.cpp 
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/Arthur/Documents/PlatformIO/Projects/Tipibot/simulator/FakeArduino/FakeArduino.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/FakeArduino_FakeArduino.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/FakeArduino_FakeArduino.cpp$(PreprocessSuffix): FakeArduino/FakeArduino.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/FakeArduino_FakeArduino.cpp$(PreprocessSuffix) FakeArduino/FakeArduino.cpp

$(IntermediateDirectory)/FakeArduino_SD.cpp$(ObjectSuffix): FakeArduino/SD.cpp 
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/Arthur/Documents/PlatformIO/Projects/Tipibot/simulator/FakeArduino/SD.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/FakeArduino_SD.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/FakeArduino_SD.cpp$(PreprocessSuffix): FakeArduino/SD.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/FakeArduino_SD.cpp$(PreprocessSuffix) FakeArduino/SD.cpp

$(IntermediateDirectory)/FakeArduino_fake_serial.cpp$(ObjectSuffix): FakeArduino/fake_serial.cpp 
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/Arthur/Documents/PlatformIO/Projects/Tipibot/simulator/FakeArduino/fake_serial.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/FakeArduino_fake_serial.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/FakeArduino_fake_serial.cpp$(PreprocessSuffix): FakeArduino/fake_serial.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/FakeArduino_fake_serial.cpp$(PreprocessSuffix) FakeArduino/fake_serial.cpp

$(IntermediateDirectory)/FakeArduino_AccelStepper.cpp$(ObjectSuffix): FakeArduino/AccelStepper.cpp 
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/Arthur/Documents/PlatformIO/Projects/Tipibot/simulator/FakeArduino/AccelStepper.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/FakeArduino_AccelStepper.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/FakeArduino_AccelStepper.cpp$(PreprocessSuffix): FakeArduino/AccelStepper.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/FakeArduino_AccelStepper.cpp$(PreprocessSuffix) FakeArduino/AccelStepper.cpp

$(IntermediateDirectory)/easywsclient.cpp$(ObjectSuffix): easywsclient.cpp 
	$(CXX) $(IncludePCH) $(SourceSwitch) "/Users/Arthur/Documents/PlatformIO/Projects/Tipibot/simulator/easywsclient.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/easywsclient.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/easywsclient.cpp$(PreprocessSuffix): easywsclient.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/easywsclient.cpp$(PreprocessSuffix) easywsclient.cpp

##
## Clean
##
clean:
	$(RM) -r ./Debug/


