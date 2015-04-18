class generic : public DAIAlgFG {
        public:
            ENUM(UpdateType,SEQFIX,SEQRND,SEQMAX,PARALL)

        protected:
            struct {
                UpdateType updates;
                double     tol;
                size_t     maxiter;
                size_t     verbose;
                double     damping;
            } Props;
            /// Maximum difference encountered so far
            double                  _maxdiff;
            /// Number of iterations needed
            size_t                  _iterations;


// DAIAlgFG interface 

        public:
            /// Default constructor
            generic() : DAIAlgFG(), Props(), _maxdiff(0.0), _iterations(0UL) {};
            
            /// Construct generic object using the specified properties
            generic( const FactorGraph & fg, const Properties &opts );
            
            /// Copy constructor
            generic( const generic & x ) : DAIAlgFG(x), Props(x.Props), _maxdiff(x._maxdiff), _iterations(x._iterations) {};

            /// Assignment operator
            generic & operator=( const generic & x ) {
                if( this != &x ) {
                    DAIAlgFG::operator=( x );
                    Props        = x.Props;
                    _maxdiff     = x._maxdiff;
                    _iterations  = x._iterations;
                }
                return *this;
            }

            /// Clone (virtual copy constructor)
            virtual generic* clone() const {
                return new generic(*this);
            }

            /// Create (virtual constructor)
            virtual generic* create() const {
                return new generic();
            }
            
            /// Return verbosity level
            virtual size_t Verbose() const {
                return Props.verbose;
            }

            /// Return number of passes over the factorgraph
            virtual size_t Iterations() const {
                return _iterations;
            }

            /// Return maximum difference between single node 
            /// beliefs for two consecutive iterations
            virtual double maxDiff() const {
                return _maxdiff;
            }

            /// Identifies itself for logging purposes
            virtual std::string identify() const;

            /// Get single node belief
            virtual Factor belief( const Var &n ) const;

            /// Get general belief
            virtual Factor belief( const VarSet &n ) const;

            /// Get all beliefs
            virtual std::vector<Factor> beliefs() const;

            /// Get log partition sum
            virtual Complex logZ() const;

            /// Clear messages and beliefs
            virtual void init();

            /// Clear messages and beliefs corresponding to the nodes in ns
            virtual void init( const VarSet &ns );

            /// The actual approximate inference algorithm
            virtual double run();

            /// Checks whether all necessary properties have been set
            /// and casts string-valued properties to other values if necessary
            virtual bool initProps();

            /// Name of this inference method
            static const char *Name;


// generic specific stuff
