#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[])
{
	FILE *in = fopen(argv[1],"r");
	char buf[1024];
	unsigned char *p;
	int t, skip;
	int tm = -1;
	int tStart = 0;
	int nSkipped = 0;
	int bOverflow = 0;

	if( !in ) {
		printf("Can't open file!\n");
		return -1;
	}
	printf("Check if file <%s> is a correct log\n", argv[1]);
	while(fgets(buf, 1024, in)) {
		p = &buf[0];
		while( *p && (*p < ' '))
			p++;
		if( strlen(p) < 5 )
			continue;
		skip = t = 0;
		if( p[0] == '#' && p[1] == '#' ) {
			if( tm == -1 )
				continue;
			sscanf(p, "###### SKIPPED %d TRACE ", &skip);
			tm += skip;
		} else {
			sscanf(p+3, "%X", &t);
			if( tm == -1 ) {
				tStart = t;
				tm = t-1;
			}
			tm++;
		}
		if( p[0] == '#' && p[1] != '#' )
			bOverflow++;
	
		//tm &= 0xFFFF;
		if( skip ) {
			nSkipped += skip;
			//printf("Skipped %d\n", skip);
		} else {
			if( t != (tm & 0xFFFF) ) {
				printf("%04X -- %04X\n", t, tm);
			}
		}
	}
	printf("\nSummary\n==========================\n");
	printf(" %d instructions logged\n", tm - tStart);
	printf("   ~ %.2f seconds of logging\n", (tm - tStart) * 160e-6);
	printf(" %d instructions skipped\n", nSkipped);
	if( bOverflow ) {
		printf("\nWarning: Overflow detected!!\n");
	}
	if( t != (tm & 0xFFFF) ) {
  	printf("%04X -- %04X\n", t, tm);
		printf("\nError: Some instructions missed!!\n");
	}
	return 0;
}
