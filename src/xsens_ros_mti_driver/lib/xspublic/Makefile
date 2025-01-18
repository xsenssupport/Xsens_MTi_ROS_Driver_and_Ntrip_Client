OBJLIBS	= libxscontroller libxscommon libxstypes

all : $(OBJLIBS)

libxscontroller : libxscommon
	$(MAKE) -C xscontroller $(MFLAGS)

libxscommon : libxstypes
	$(MAKE) -C xscommon $(MFLAGS)

libxstypes :
	$(MAKE) -C xstypes $(MFLAGS) libxstypes.a

clean :
	-$(MAKE) -C xscontroller $(MFLAGS) clean
	-$(MAKE) -C xscommon $(MFLAGS) clean
	-$(MAKE) -C xstypes $(MFLAGS) clean
