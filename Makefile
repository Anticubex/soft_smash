#Compiler and Linker
CC		  := gcc

#The Target Binary Program
TARGET	  := soft_smash

#Debugging stuff
DEBUG	:= GDB
BREAKPOINTS := bp

#The Directories, Source, Includes, Objects, Binary and Resources
SRCDIR	  := src
INCDIR	  := inc
LIBDIR	  := lib
BUILDDIR	:= obj
TARGETDIR   := bin
RESDIR	  := res
SRCEXT	  := c
DEPEXT	  := d
OBJEXT	  := o

#Flags, Libraries and Includes
CFLAGS	  := -Wall -g
LIB		 := -L$(LIBDIR) -lraylib -lgdi32 -lwinmm -ltess2
INC		 := -I$(INCDIR)
INCDEP	  := -I$(INCDIR)

#---------------------------------------------------------------------------------
#DO NOT EDIT BELOW THIS LINE
#---------------------------------------------------------------------------------
SOURCES	 := $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
OBJECTS	 := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.$(OBJEXT)))

#Default Make
all: resources $(TARGET)

#Remake
remake: cleaner all

#Copy Resources from Resources Directory to Target Directory
resources: directories
	@cp -R $(RESDIR)/ $(TARGETDIR)/

#Make the Directories
directories:
	@mkdir -p $(TARGETDIR)
	@mkdir -p $(BUILDDIR)

#Clean only Objecst
clean:
	@$(RM) -rf $(BUILDDIR)

#Full Clean, Objects and Binaries
cleaner: clean
	@$(RM) -rf $(TARGETDIR)

# #Pull in dependency info for *existing* .o files
# ifneq ($(MAKECMDGOALS),clean)
#     -include $(OBJECTS:.$(OBJEXT)=.$(DEPEXT))
# else ifneq ($(MAKECMDGOALS),run)
#     -include $(OBJECTS:.$(OBJEXT)=.$(DEPEXT))
# endif

$(BREAKPOINTS):
	touch $(BREAKPOINTS)

gdb: $(TARGET) $(BREAKPOINTS)
	gdb $(TARGETDIR)/$(TARGET).exe -x $(BREAKPOINTS)

#Link
$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(TARGETDIR)/$(TARGET).exe $^ $(LIB)

#Compile
$(BUILDDIR)/%.$(OBJEXT): $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<
	@#$(CC) $(CFLAGS) $(INCDEP) -MM $(SRCDIR)/$*.$(SRCEXT) > $(BUILDDIR)/$*.$(DEPEXT)

run:
	$(TARGETDIR)/$(TARGET).exe

#Non-File Targets
.PHONY: all remake clean cleaner resources gdb