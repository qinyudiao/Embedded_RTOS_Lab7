// filename ************** eFile.c *****************************
// High-level routines to implement a solid-state disk
// Jonathan W. Valvano 3/9/17

#include <string.h>
#include "eDisk.h"
#include "UART.h"
#include <stdio.h>
#include <stdbool.h>

#define SUCCESS 0
#define FAIL 1

// Sector size in bytes
#define SECTOR_BYTES 512

typedef uint32_t sector_addr_t;

#define LONGEST_FILENAME 7 // Excluding null terminator
typedef struct
{
  char file_name[LONGEST_FILENAME + 1]; // space for null terminator
  sector_addr_t start;
  uint32_t size; // size of zero indicates unused, size increase as write. size-1 is the point to write or read.
} dir_entry_t;

typedef struct
{
  int dir_idx;
  int sectornum; // Sector offset from DATA_START
  int bytenum;
} open_file_metadata_t;

// Sectors 0 through 31 are reserved
#define DIR_START 32  // Sector number
#define DIR_SECTORS 2 // Number of sectors directory may span
#define DIR_ENTRIES ((DIR_SECTORS * SECTOR_BYTES) / (sizeof(dir_entry_t)))
static dir_entry_t dir[DIR_ENTRIES];

#define FAT_SECTORS (130056 / 32) // Number of sectors FAT may span
#define FAT_ENTRIES ((FAT_SECTORS * SECTOR_BYTES) / sizeof(sector_addr_t))
#define FAT_START 34
#define CACHED_SECTORS (SECTOR_BYTES / sizeof(sector_addr_t)) //

#define DATA_START 130090

static sector_addr_t fat_cache[CACHED_SECTORS];
static sector_addr_t cached_fat_sector = -1; // Sector offset from start of FAT
static sector_addr_t last_free_sector = 0; // last freesector 

static bool fat_cache_dirty = false;
static bool write_mode = false;
static BYTE DATAarray[SECTOR_BYTES];
static open_file_metadata_t open_file;

void cache_fat_sector(sector_addr_t fat_sector_offset);
int eFile_WClose(void);
int eFile_RClose(void);

static int get_writepoint_in_sector(int dir_idx)
{
  return dir[dir_idx].size - 1;
}

static void cache_file_sector(sector_addr_t data_sector_offset)
{
  eDisk_ReadBlock((BYTE *)DATAarray, DATA_START + data_sector_offset);
}

static void writeback_file_sector(void)
{
  eDisk_WriteBlock((BYTE *)DATAarray, DATA_START + open_file.sectornum);
}

/**
 * @brief Get the next sector of a file.
 * 
 * @param data_sector_offset Sector offset from DATA_START to lookup in the FAT.
 * @return sector_addr_t Return value is interpreted as an offset from DATA_START.
 */
static sector_addr_t get_next_file_sector(sector_addr_t data_sector_offset)
{
  sector_addr_t fat_sector = data_sector_offset / CACHED_SECTORS;
  cache_fat_sector(fat_sector);
  return fat_cache[data_sector_offset % CACHED_SECTORS];
}

static sector_addr_t get_last_file_sector(sector_addr_t start_data_sector_offset)
{
  sector_addr_t start = start_data_sector_offset;
  sector_addr_t end = start;
  sector_addr_t next = start;
  do 
  {
    end = next;
    next = get_next_file_sector(end);
		if(end==next)
			break;
  } while(next != -1);
  return end;
}

static void writeback_fat_cache(void)
{
  eDisk_WriteBlock((BYTE *)fat_cache, FAT_START + cached_fat_sector);
  fat_cache_dirty = false;
}

static int lookup_file_dir_idx(char name[])
{
  // Skip idx 0 where free space is tracked
  for (int i = 1; i < DIR_ENTRIES; i++)
  {
    if ((dir[i].size > 0) && (strncmp(name, dir[i].file_name, LONGEST_FILENAME) == 0))
    {
      return i;
    }
  }
  return -1;
}

 void cache_fat_sector(sector_addr_t fat_sector_offset)
{
	int tmp =0;
  if (cached_fat_sector != fat_sector_offset)
  {
    if (fat_cache_dirty)
    {
      writeback_fat_cache();
    }
    cached_fat_sector = fat_sector_offset;
    eDisk_ReadBlock((BYTE *)fat_cache, FAT_START + fat_sector_offset);
		
  }
}

static void writeback_dir(void)
{
  for (int i = 0; i < DIR_SECTORS; i++)
  {
    eDisk_WriteBlock(((uint8_t *)dir) + i * SECTOR_BYTES, DIR_START + i);
  }
}

static void cache_dir(void)
{
  for (int i = 0; i < DIR_SECTORS; i++)
  {
    eDisk_ReadBlock(((uint8_t *)dir) + i * SECTOR_BYTES, DIR_START + i);
  }
}

int eFile_Init(void)
{
  memset(dir, 0, sizeof(dir));
  memset(fat_cache, 0, sizeof(fat_cache));
  eDisk_Init(0);
  cache_dir();
  cache_fat_sector(0);
	last_free_sector = get_last_file_sector(dir[0].start);
  return SUCCESS;
}

int eFile_Format(void)
{
	if(open_file.dir_idx != 0) // if some file was opened, we can't do format.
		return FAIL;

  memset(dir, 0, sizeof(dir));
  memset(fat_cache, 0, sizeof(fat_cache));
	
  for (int i = 0; i < FAT_SECTORS; i++)
  {
    cached_fat_sector = i; // Offset from start of FAT in sectors
    for (int j = 0; j < CACHED_SECTORS; j++)
    {
      fat_cache[j] = i * CACHED_SECTORS + j + 1;
    }
    writeback_fat_cache();
  }
	fat_cache[CACHED_SECTORS-1] = -1;
	last_free_sector = FAT_SECTORS*CACHED_SECTORS-1;
	writeback_fat_cache();
  dir[0].start = 0; // sector offset from DATA_START
  dir[0].size = 1;

  writeback_dir();
  return SUCCESS;
}

int eFile_Create(char name[])
{
  dir_entry_t new_file;
  int already_exists_at_idx = lookup_file_dir_idx(name);
  if (already_exists_at_idx != -1)
    return FAIL; // File already exists with this name

  // Find empty dir entry to use
  int free_idx;
  // Skip idx 0 where free space is tracked
  for (free_idx = 1; free_idx < DIR_ENTRIES; free_idx++)
  {
    if (dir[free_idx].size == 0)
      break; // Free dir entry at idx, we will allocate it to this file
  }
  if (free_idx == DIR_ENTRIES)
    return FAIL; // Directory is full, cannot create new file

  int freespace_head = dir[0].start;                         // get free space that we could use
  cache_fat_sector(freespace_head / CACHED_SECTORS);         // Read sector of FAT from disk
  dir[0].start = fat_cache[freespace_head % CACHED_SECTORS]; // set head of free space to nextone. (Next freespace using linked list kind of work)

  // Create directory entry with given name.
  strncpy(dir[free_idx].file_name, name, LONGEST_FILENAME);
  dir[free_idx].size = 1; // set size to indicate entry occupied. True size of file is dir[].size - 1
  dir[free_idx].start = freespace_head;

  fat_cache[freespace_head % CACHED_SECTORS] = -1; // Set this sector as end of new file in FAT
  fat_cache_dirty = true;

  writeback_fat_cache();
  writeback_dir();
  return SUCCESS;
}

int eFile_WOpen(char name[])
{
	if(open_file.dir_idx != 0) // if some file is already opened
		return FAIL;
	
  int FAT_sec_offset; // Sector offset from base sector of FAT
  int iter = 0;
  int prev_iter = 0;
  int open_idx = lookup_file_dir_idx(name);
  if (open_idx == -1)
    return FAIL; // There is no file to open
  write_mode = true;
  FAT_sec_offset = dir[open_idx].start / CACHED_SECTORS;
  prev_iter = dir[open_idx].start;
  cache_fat_sector(FAT_sec_offset);
  iter = fat_cache[prev_iter % CACHED_SECTORS];
  while (iter != -1)
  {
    FAT_sec_offset = iter / CACHED_SECTORS;
    cache_fat_sector(FAT_sec_offset);
    prev_iter = iter;
    iter = fat_cache[prev_iter % CACHED_SECTORS];
  }
  //write_point_in_sector = (dir[open_idx].size - 1)%SECTOR_BYTES;
  eDisk_ReadBlock(DATAarray, prev_iter + DATA_START);
  open_file.bytenum = (dir[open_idx].size - 1);
  open_file.dir_idx = open_idx;
  open_file.sectornum = prev_iter;

  return SUCCESS;
}

int eFile_Write(char data)
{
  if (!write_mode || open_file.dir_idx == 0)
    return FAIL;

  int w_idx = open_file.bytenum;
  if ((w_idx != 0) && (w_idx % SECTOR_BYTES == 0))
  {
    writeback_file_sector();
    int freespace = dir[0].start; // get free space that we could use
    fat_cache[open_file.sectornum % CACHED_SECTORS] = freespace;

    //need to change fatcache to freespace fatcache
    cache_fat_sector(freespace / CACHED_SECTORS);
    dir[0].start = fat_cache[freespace % CACHED_SECTORS]; // set head of free space to nextone. (Next freespace using linked list kind of work)
    fat_cache[freespace % CACHED_SECTORS] = -1;           // This is creating part, make sure currently allocated space is last one.
    fat_cache_dirty = true;
    open_file.sectornum = freespace;
    cache_file_sector(freespace);
  }
  DATAarray[w_idx % SECTOR_BYTES] = data;
  open_file.bytenum++;
  return SUCCESS;
}
int eFile_Close(void)
{
	if(open_file.dir_idx == 0)
		return FAIL;
	//Nothing was opened
	
	if(write_mode)
		eFile_WClose();
	else
		eFile_RClose();
	return SUCCESS;
}

int eFile_WClose(void)
{
		if(open_file.dir_idx == 0)
			return FAIL;
	//Nothing was opened
		
  dir[open_file.dir_idx].size = open_file.bytenum + 1;
  writeback_dir();
  writeback_fat_cache();
  writeback_file_sector();
  write_mode = false;
  memset(&open_file, 0, sizeof(open_file));
  return SUCCESS;
}

int eFile_ROpen(char name[])
{
	if(open_file.dir_idx != 0)
		return FAIL;
  int FAT_sec_offset; // Sector offset from base sector of FAT
  int open_idx = lookup_file_dir_idx(name);
  if (open_idx == -1)
    return FAIL;
  write_mode = false;
  open_file.dir_idx = open_idx;
  open_file.bytenum = 0;
  open_file.sectornum = dir[open_idx].start;
  cache_file_sector(open_file.sectornum);
  return SUCCESS;
}

static bool read_reached_eof(void)
{
  return (open_file.bytenum >= (dir[open_file.dir_idx].size - 1));
}

int eFile_ReadNext(char *pt)
{
  if (write_mode == false)
  {
    if (!read_reached_eof())
    {
      // More data left to read
      *pt = DATAarray[open_file.bytenum++ % SECTOR_BYTES];
      if ((open_file.bytenum % SECTOR_BYTES == 0) && !read_reached_eof())
      {
        // Get next sector
        open_file.sectornum = get_next_file_sector(open_file.sectornum);
        cache_file_sector(open_file.sectornum);
      }
      return SUCCESS;
    }
  }
  return FAIL;
}

int eFile_RClose(void)
{
		if(open_file.dir_idx == 0)
			return FAIL;
	//Nothing was opened
		
  // No fsync necessary, file should be unchanged in RAM
  memset(&open_file, 0, sizeof(open_file));
  return SUCCESS;
}

int eFile_Directory(void (*fp)(char))
{
	if(open_file.dir_idx != 0)
		return FAIL;
  for (int i = 1; i < DIR_ENTRIES; i++)
  {
    if (dir[i].size > 0)
    {
      char file_desc[64];
      char sectors_str[32];
      memset(file_desc, 0, sizeof(file_desc));
      memset(sectors_str, 0, sizeof(sectors_str));
      sector_addr_t next = dir[i].start;
      unsigned int str_idx = 0;
      str_idx += snprintf(sectors_str+str_idx, sizeof(sectors_str)-str_idx, "%lu", next);
      next = get_next_file_sector(next);
      while((next != -1) && (str_idx < sizeof(sectors_str)))
      {
        str_idx += snprintf(sectors_str+str_idx, sizeof(sectors_str)-str_idx, ", %lu", next);
        next = get_next_file_sector(next);
      }
      snprintf(file_desc, sizeof(file_desc), "%s: %dB sectors %s\r\n", dir[i].file_name, dir[i].size - 1, sectors_str);
      char *c = file_desc;
      while (*c != 0)
      {
        fp(*c);
				c++;
      }
    }
  }
	return SUCCESS;
}

int eFile_Delete(char name[])
{
	if(open_file.dir_idx != 0)
		return FAIL;
  int del_idx  = lookup_file_dir_idx(name);
	
  if (del_idx == -1)
    return FAIL;

  // Attach head of file's sector list to the tail of the free space linked list
  sector_addr_t last_free = last_free_sector; // Free space head
  cache_fat_sector(last_free / CACHED_SECTORS);
  fat_cache[last_free % CACHED_SECTORS] = dir[del_idx].start; // freespace tail --> deleted file head
  last_free_sector = get_last_file_sector(dir[del_idx].start);
	
  // Erase directory entry
  memset(dir[del_idx].file_name, 0, sizeof(dir[0].file_name));
  dir[del_idx].size = 0;
  dir[del_idx].start = 0;

  writeback_fat_cache();
  writeback_dir();
	return SUCCESS;
}

int eFile_RedirectToFile(char *name)
{
	UART_setRedirect(name);
	return SUCCESS;
}

int eFile_EndRedirectToFile(void)
{
	UART_endRedirect();
	return SUCCESS;
}
