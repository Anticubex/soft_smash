#Compiler and Linker
CC		  := gcc

#The Target Binary Program
TARGET	  := soft_smash

#The Directories, Source, Includes, Objects, Binary and Resources
SRCDIR	  := src
INCDIR	  := inc
BUILDDIR	:= obj
TARGETDIR   := bin
RESDIR	  := res
SRCEXT	  := c
DEPEXT	  := d
OBJEXT	  := o

#Flags, Libraries and Includes
PRODFLAGS := -Wall -O3
CFLAGS	  := -Wall -g
LIB		 := -lraylib -lgdi32 -lwinmm
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

#Pull in dependency info for *existing* .o files
ifneq ($(MAKECMDGOALS),clean)
    -include $(OBJECTS:.$(OBJEXT)=.$(DEPEXT))
endif

#Link
$(TARGET): $(OBJECTS)
	$(CC) -o $(TARGETDIR)/$(TARGET).exe $^ $(LIB)

#Compile
$(BUILDDIR)/%.$(OBJEXT): $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(dir $@)
	@$(CC) $(CFLAGS) $(INCDEP) -MM $(SRCDIR)/$*.$(SRCEXT) > $(BUILDDIR)/$*.$(DEPEXT)
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<
	@sed -e ':a;N;$$!ba;s/ \\\\\\n//g' < $(BUILDDIR)/$*.$(DEPEXT) > $(BUILDDIR)/$*.$(DEPEXT).tmp # Account for gcc line-breaking long dependency files
	@sed -e 's|.*:|$(BUILDDIR)/$*.$(OBJEXT):|' < $(BUILDDIR)/$*.$(DEPEXT).tmp > $(BUILDDIR)/$*.$(DEPEXT) # stick the builddir stem on the filename
	@sed -e 's/.*://' -e 's/\\$$//' < $(BUILDDIR)/$*.$(DEPEXT).tmp | fmt -1 | sed -e 's/^ *//' -e 's/$$/:/' >> $(BUILDDIR)/$*.$(DEPEXT) # split and add the files into new lines
	@rm -f $(BUILDDIR)/$*.$(DEPEXT).tmp

run:
	$(TARGETDIR)/$(TARGET).exe

#Non-File Targets
.PHONY: all remake clean cleaner resources