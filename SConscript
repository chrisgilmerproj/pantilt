Import('env')

sourceList = ['libraries', 'programs', ]

#-------------------------------------------------------------------------------
# Nothing to modify below this line
#-------------------------------------------------------------------------------

for sourceName in sourceList:
   SConscript('./%s/SConscript' % sourceName)

Export('env')