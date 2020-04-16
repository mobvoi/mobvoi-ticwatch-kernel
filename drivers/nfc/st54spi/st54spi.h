struct mtk_chip_config {
	int rx_mlsb;
	int tx_mlsb;
	int cs_pol;
	int sample_sel;

	int cs_idletime;
	int cs_setuptime;
	int cs_holdtime;

};
#define ST54SPI_READ_BUF_SIZE	(8)	//*PAGE_SIZE)
#define ST54SPI_WRITE_BUF_SIZE	(8)	//*PAGE_SIZE)

struct st54spi_tx_frame {
	unsigned char cmd;
	unsigned char data[ST54SPI_WRITE_BUF_SIZE - 1];
} __attribute__ ((__packed__));
//__attribute__used for set the attribute of the Function
//((__packed__)) used for insist particular sized padding, avoid data alignment

struct st54spi_rx_frame {
	unsigned char status;
	unsigned char len;
	unsigned char data[ST54SPI_READ_BUF_SIZE - 2];
} __attribute__ ((__packed__));

struct st54spi_priv {
	struct spi_device *spi;

	/* Char device stuff */
	//struct miscdevice misc;
	bool busy;
	//struct circ_buf read_buf;
	//struct circ_buf write_buf;
	struct mutex rlock;	/* Lock for read_buf */
	struct mutex wlock;	/* Lock for write_buf */
	char _read_buf[ST54SPI_READ_BUF_SIZE];
	char _write_buf[ST54SPI_WRITE_BUF_SIZE];
	wait_queue_head_t poll_wait;	/* for poll */

	/* IRQ and its control */
	atomic_t irq_enabled;
	spinlock_t irq_lock;

	/* Work */
	struct work_struct rxtx_work;
	struct workqueue_struct *serial_wq;
	atomic_t suspending;

	/* SPI tx/rx buf */
	struct st54spi_tx_frame *tx_buf;
	struct st54spi_rx_frame *rx_buf;

//      struct wake_lock st54spi_wake_lock;
};
