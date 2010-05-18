INSTALL_DIR ?= /vol/nirobots
CXXFLAGS ?= -O4 -mfpmath=sse -march=native -mtune=native
#CXXFLAGS ?= -O4 -mfpmath=sse -march=pentium3
#CXXFLAGS ?= -O0 -g3

BUILD=o.$(ARCH)

.PHONY : all clean veryclean depend install

all: $(BUILD) depend libSTP.so stp.so

doc: 
	doxygen

clean:
	rm -f $(BUILD)/*.o

veryclean: clean
	rm -f libSTP.so stp.so

SOURCES=complex.cc polynomial.cc tools.cc \
	     stp1.cc stp3.cc stp7.cc stp7Formulars.cc STP.cc MultiSTP.cc

libSTP.so: CXXFLAGS += -I$(INSTALL_DIR)/include -I/vol/xcf/include -DUSE_LOG4CXX -fPIC
libSTP.so: $(SOURCES:%.cc=$(BUILD)/%.o)
	$(CXX) -shared $^ -o $(@F) -L$(INSTALL_DIR)/lib -lLogger

libSTP.a: CXXFLAGS += -I$(INSTALL_DIR)/include -I/vol/xcf/include -DUSE_LOG4CXX
libSTP.a: $(SOURCES:%.cc=$(BUILD)/%.o)
	ar rcs $(@F) $^

stp.so: LDFLAGS += -L. -L$(INSTALL_DIR)/lib -Wl,-rpath -Wl,$(INSTALL_DIR)/lib -lSTP
stp.so: CXXFLAGS += -I/vol/nst/include -Wno-write-strings -fPIC
stp.so: $(BUILD)/progaux_stp.o
	$(CXX) -shared $^ -o $(@F) $(LDFLAGS)

progaux_stp.o: CXXFLAGS += -I$(STPDIR)/include


INCLUDE_DIR=$(INSTALL_DIR)/include/STP
LIB_DIR=$(INSTALL_DIR)/lib

install: all
	rm -f $(INCLUDE_DIR)/*
	install -d $(INCLUDE_DIR)
	install --mode=444 $(SOURCES:%.cc=%.hh) stp.hh $(INCLUDE_DIR)
	install --mode=755 libSTP.so $(LIB_DIR)
	install --mode=775 stp.so /vol/nst/lib

$(BUILD)/*.o: makefile

$(BUILD):
	mkdir $(BUILD)

depend: $(SOURCES:%.cc=$(BUILD)/%.d)

$(BUILD)/%.d: %.c
	@echo "generating dependencies for file $<"
	@$(CC) $(CFLAGS) $< -MM -o $@
	@sed --in-place 's#.*\.o:#$(BUILD)/&#' $@

$(BUILD)/%.d: %.cc
	@echo "generating dependencies for file $<"
	@$(CXX) $(CXXFLAGS) $< -MM -o $@
	@sed --in-place 's#.*\.o:#$(BUILD)/&#' $@

$(BUILD)/%.d: %.cpp
	@echo "generating dependencies for file $<"
	@$(CXX) $(CXXFLAGS) $< -MM -o $@
	@sed --in-place 's#.*\.o:#$(BUILD)/&#' $@

$(BUILD)/%.o: %.cc 
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

-include $(wildcard $(BUILD)/*.d)
