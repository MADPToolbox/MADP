#replace HEADER:
        /\/* REPLACE_MADP_HEADER/{        
            r Template_HEADER
            d
        }
#replace REPLACE_CONTRIBUTING_AUTHORS_START:
        /\/* REPLACE_CONTRIBUTING_AUTHORS_START/{
            r Template_CONTRIBUTING_AUTHORS_START
            d
        }
#replace REPLACE_CONTRIBUTING_AUTHORS_END:        
        /\/* REPLACE_CONTRIBUTING_AUTHORS_END/{
            r Template_CONTRIBUTING_AUTHORS_END
            d
        }
#remove emails:
        s/<[a-z,.]*@[a-z,.]*>//
